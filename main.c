#include "raylib.h"
#define RAYGUI_IMPLEMENTATION
#include "raygui.h"

#include "simulation.h"
#include "gui.h"
#include "config.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

int main(void) {
    // --- Initialization ---

    // Allocate memory for the simulation state on the heap
    SimulationState *simState = (SimulationState*)malloc(sizeof(SimulationState));
    if (simState == NULL) {
        TraceLog(LOG_FATAL, "Failed to allocate memory for SimulationState!");
        return 1;
    }
    // Set up initial simulation parameters (boids, rules, dimensions, etc.)
    Simulation_Initialize(simState);

    // Initialize raylib window and set target frame rate
    InitWindow(simState->screenWidth, simState->screenHeight, "3D Boids Simulation");
    ToggleFullscreen(); // Start in fullscreen mode
    SetTargetFPS(60);

    // Setup the 3D camera
    Camera3D camera = { 0 };
    camera.position = (Vector3){ simState->cubeDimensions.x / 2.0f,
                                 simState->cubeDimensions.y / 1.5f,
                                 simState->cubeDimensions.z * 2.5f }; // Position looking at the cube
    camera.target = (Vector3){ simState->cubeDimensions.x / 2.0f,
                               simState->cubeDimensions.y / 2.0f,
                               simState->cubeDimensions.z / 2.0f }; // Look at the center of the cube
    camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };
    camera.fovy = 50.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    // Create a render texture to draw the 3D simulation view into
    int simulationViewWidth = GetScreenWidth() - PANEL_WIDTH;
    int simulationViewHeight = GetScreenHeight();
    RenderTexture2D simulationTexture = LoadRenderTexture(simulationViewWidth, simulationViewHeight);
    SetTextureFilter(simulationTexture.texture, TEXTURE_FILTER_BILINEAR);

    // Setup GUI layout variables
    GuiSetStyle(DEFAULT, TEXT_SIZE, GUI_CONTROLS_TEXT_SIZE);
    Rectangle panelRec = { (float)simulationViewWidth, 0, PANEL_WIDTH, (float)simulationViewHeight };
    float sliderStartX = (float)simulationViewWidth + GUI_PADDING;
    float sliderWidth = (float)PANEL_WIDTH - (GUI_PADDING * 2.0f);
    if (sliderWidth < 100) sliderWidth = 100;

    // --- Main Game Loop ---
    while (!WindowShouldClose()) {
        // --- Input Handling ---

        // Toggle camera mode based on GUI checkbox
        if (simState->auto_rotate_camera) {
            UpdateCamera(&camera, CAMERA_ORBITAL); // Automatic orbital movement
        } else {
        }

        // Toggle simulation pause state
        if (IsKeyPressed(KEY_P)) {
            Simulation_TogglePause(simState);
        }

        // Handle window resizing
        if (IsWindowResized()) {
            // Update dimensions used for render texture and GUI panel
            simulationViewWidth = GetScreenWidth() - PANEL_WIDTH;
            simulationViewHeight = GetScreenHeight();

            UnloadRenderTexture(simulationTexture);
            simulationTexture = LoadRenderTexture(simulationViewWidth, simulationViewHeight);
            SetTextureFilter(simulationTexture.texture, TEXTURE_FILTER_BILINEAR);

            panelRec = (Rectangle){ (float)simulationViewWidth, 0, PANEL_WIDTH, (float)simulationViewHeight };
            sliderStartX = (float)simulationViewWidth + GUI_PADDING;
            sliderWidth = (float)PANEL_WIDTH - (GUI_PADDING * 2.0f);
            if (sliderWidth < 100) sliderWidth = 100;
        }

        // --- Update ---
        // Update the boid simulation state (movement, interactions)
        Simulation_Update(simState);

        // --- Drawing ---

        // Draw the 3D simulation into the render texture
        BeginTextureMode(simulationTexture);
        {
            ClearBackground((Color){ 30, 43, 56, 255 }); // Dark background for 3D view

            BeginMode3D(camera);
            {
                // Draw the bounding box wireframe
                Vector3 cubeCenter = { simState->cubeDimensions.x / 2.0f,
                                       simState->cubeDimensions.y / 2.0f,
                                       simState->cubeDimensions.z / 2.0f };
                DrawCubeWiresV(cubeCenter, simState->cubeDimensions, GRAY);

                // Draw all the active boids
                Simulation_DrawBoids(simState, camera);
            }
            EndMode3D();
        }
        EndTextureMode();

        // Draw the main window content (simulation texture and GUI)
        BeginDrawing();
        {
            ClearBackground((Color){ 60, 63, 65, 255 }); // Dark background for window border/GUI

            // Draw the 3D simulation view (rendered to texture)
            DrawTextureRec(simulationTexture.texture,
                           (Rectangle){ 0, 0, (float)simulationTexture.texture.width, -(float)simulationTexture.texture.height }, // Source rec (flip Y)
                           (Vector2){ 0, 0 }, // Position
                           WHITE); // Tint

            // Draw the GUI panel on the right side
            GUI_DrawTabsAndPanel(simState, &panelRec, sliderStartX, sliderWidth);

            // Draw FPS counter
            DrawFPS(10, 10);

            // Draw pause indicator text if paused
            if (simState->sim_paused) {
                const char* pauseText = "PAUSED";
                int fontSize = 40;
                float textWidth = MeasureText(pauseText, fontSize);
                DrawText(pauseText,
                         simulationViewWidth / 2 - (int)(textWidth / 2.0f),
                         simulationViewHeight / 2 - fontSize / 2,
                         fontSize,
                         YELLOW);
            }
        }
        EndDrawing();
    }

    // --- De-Initialization ---
    UnloadRenderTexture(simulationTexture); // Unload the render texture
    Simulation_Cleanup(simState); // Clean up simulation resources (boids array, grid data)
    free(simState); // Free the simulation state memory

    CloseWindow(); // Close raylib window
    return 0;
}
