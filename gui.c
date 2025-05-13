#include "gui.h"
#include "raylib.h"
#include "raymath.h"
#include "config.h"
#include "utils.h"
#include "simulation.h"
#include <stdio.h>
#include <math.h>

#include "raygui.h"

// Simple helper to draw text centered horizontally within a given container.
void DrawCenteredText(const char *text, float y, float containerWidth, float containerX, int fontSize, Color color) {
    float textWidth = MeasureText(text, fontSize);
    DrawText(text, (int)(containerX + (containerWidth - textWidth) / 2.0f), (int)y, fontSize, color);
}

// Applies a dark theme style to raygui controls.
// These colors are set to match the overall application aesthetic.
void GUI_SetDarkStyle() {
    GuiSetStyle(DEFAULT, TEXT_COLOR_NORMAL,   ColorToInt(LIGHTGRAY));
    GuiSetStyle(DEFAULT, TEXT_COLOR_FOCUSED,   ColorToInt(WHITE));
    GuiSetStyle(DEFAULT, TEXT_COLOR_PRESSED,   ColorToInt(WHITE));
    GuiSetStyle(DEFAULT, TEXT_COLOR_DISABLED,  ColorToInt(GRAY));

    GuiSetStyle(DEFAULT, BORDER_COLOR_NORMAL,  ColorToInt(GRAY));
    GuiSetStyle(DEFAULT, BORDER_COLOR_FOCUSED,  ColorToInt(LIGHTGRAY));
    GuiSetStyle(DEFAULT, BORDER_COLOR_PRESSED,  ColorToInt(LIGHTGRAY));
    GuiSetStyle(DEFAULT, BORDER_COLOR_DISABLED, ColorToInt(DARKGRAY));

    GuiSetStyle(DEFAULT, BASE_COLOR_NORMAL,   ColorToInt((Color){ 70, 73, 75, 255 }));
    GuiSetStyle(DEFAULT, BASE_COLOR_FOCUSED,   ColorToInt((Color){ 80, 83, 85, 255 }));
    GuiSetStyle(DEFAULT, BASE_COLOR_PRESSED,   ColorToInt((Color){ 50, 53, 55, 255 }));
    GuiSetStyle(DEFAULT, BASE_COLOR_DISABLED,  ColorToInt(DARKGRAY));

    GuiSetStyle(SLIDER, BORDER_COLOR_NORMAL, ColorToInt(GRAY));
    GuiSetStyle(SLIDER, BASE_COLOR_NORMAL, ColorToInt((Color){ 50, 53, 55, 255 }));
    GuiSetStyle(SLIDER, BASE_COLOR_PRESSED, ColorToInt(RAYWHITE));
    GuiSetStyle(SLIDER, TEXT_COLOR_NORMAL, ColorToInt(LIGHTGRAY));
}

// Calculates the initial position and dimensions of the main GUI panel
// and the usable area within it for controls like sliders.
void GUI_SetupLayout(SimulationState *sim, Rectangle *panelRec, float *sliderStartX, float *sliderWidth) {
    *panelRec = (Rectangle){ (float)GetScreenWidth() - PANEL_WIDTH, 0, PANEL_WIDTH, (float)GetScreenHeight() };
    *sliderStartX = (float)GetScreenWidth() - PANEL_WIDTH + GUI_PADDING;
    *sliderWidth = (float)PANEL_WIDTH - (GUI_PADDING * 2.0f);
    if (*sliderWidth < 100) *sliderWidth = 100;
}

// Main drawing function for the GUI panel.
// Sets the style, draws the panel background, and then delegates drawing the content.
// (Currently, only the controls tab is drawn directly as tabs were removed).
void GUI_DrawTabsAndPanel(SimulationState *sim, const Rectangle *panelRec, float sliderStartX, float sliderWidth) {
    GUI_SetDarkStyle();

    DrawRectangleRec(*panelRec, (Color){ 60, 63, 65, 255 });
    if (panelRec->x > 0) {
        DrawLine((int)panelRec->x, 0, (int)panelRec->x, GetScreenHeight(), GRAY);
    }

    Rectangle contentRec = {
            panelRec->x,
            panelRec->y + GUI_PADDING,
            panelRec->width,
            panelRec->height - GUI_PADDING * 2.0f
    };

    Rectangle tabViewArea = {
            contentRec.x + GUI_PADDING,
            contentRec.y,
            contentRec.width - 2 * GUI_PADDING,
            contentRec.height - GUI_PADDING
    };
    if (tabViewArea.width < 10) tabViewArea.width = 10;
    if (tabViewArea.height < 10) tabViewArea.height = 10;

    GUI_DrawControlsTab(sim, tabViewArea);
}

// Responsible for drawing all the controls (sliders, buttons, checkboxes)
// within the designated view area of the panel.
void GUI_DrawControlsTab(SimulationState *sim, Rectangle view) {
    Rectangle controlRec; // Reusable rectangle for positioning controls
    float currentY = view.y; // Tracks the vertical position for the next control
    float controlX = view.x;
    float controlWidth = view.width;
    float controlXCenter = view.x + view.width / 2.0f;
    const Color labelColor = LIGHTGRAY;
    const Color headerColor = WHITE;
    const int valueTextSize = GUI_LABEL_TEXT_SIZE - 2;
    const float valueTextOffsetX = 5.0f; // Small offset for slider value text
    char valueTextBuffer[32]; // Buffer for formatting slider values

    // --- Main Parameters Section --- //
    DrawCenteredText("--- Main Parameters ---", currentY, view.width, view.x, GUI_LABEL_TEXT_SIZE + 2, headerColor);
    currentY += (GUI_LABEL_TEXT_SIZE + 2) + GUI_VERTICAL_SPACING;

    DrawCenteredText("Number of Boids", currentY, view.width, view.x, GUI_LABEL_TEXT_SIZE, labelColor);
    currentY += GUI_LABEL_TEXT_SIZE + GUI_LABEL_CONTROL_SPACING;
    float temp_num_boids = (float)sim->current_num_boids;
    controlRec = (Rectangle){ controlX, currentY, controlWidth, GUI_SLIDER_HEIGHT };
    GuiSliderBar(controlRec, NULL, NULL, &temp_num_boids, 1, MAX_BOIDS);
    snprintf(valueTextBuffer, 32, "%d", sim->current_num_boids);
    DrawText(valueTextBuffer, (int)(controlRec.x + controlRec.width - MeasureText(valueTextBuffer, valueTextSize) - valueTextOffsetX),
             (int)(controlRec.y + controlRec.height / 2 - valueTextSize / 2), valueTextSize, labelColor);
    int new_boid_count = (int)temp_num_boids;
    if (new_boid_count != sim->current_num_boids) Simulation_SetBoidCount(sim, new_boid_count);
    currentY += GUI_SLIDER_HEIGHT + GUI_VERTICAL_SPACING;

    DrawCenteredText("Maximum Speed", currentY, view.width, view.x, GUI_LABEL_TEXT_SIZE, labelColor); currentY += GUI_LABEL_TEXT_SIZE + GUI_LABEL_CONTROL_SPACING;
    controlRec = (Rectangle){ controlX, currentY, controlWidth, GUI_SLIDER_HEIGHT };
    GuiSliderBar(controlRec, NULL, NULL, &sim->max_speed, 0.5f, 10.0f);
    snprintf(valueTextBuffer, 32, "%.2f", sim->max_speed);
    DrawText(valueTextBuffer, (int)(controlRec.x + controlRec.width - MeasureText(valueTextBuffer, valueTextSize) - valueTextOffsetX),
             (int)(controlRec.y + controlRec.height / 2 - valueTextSize / 2), valueTextSize, labelColor);
    currentY += GUI_SLIDER_HEIGHT + GUI_VERTICAL_SPACING;

    DrawCenteredText("Perception Radius", currentY, view.width, view.x, GUI_LABEL_TEXT_SIZE, labelColor); currentY += GUI_LABEL_TEXT_SIZE + GUI_LABEL_CONTROL_SPACING;
    float old_perception = sim->perception_radius;
    controlRec = (Rectangle){ controlX, currentY, controlWidth, GUI_SLIDER_HEIGHT };
    GuiSliderBar(controlRec, NULL, NULL, &sim->perception_radius, 5.0f, 200.0f);
    snprintf(valueTextBuffer, 32, "%.1f", sim->perception_radius);
    DrawText(valueTextBuffer, (int)(controlRec.x + controlRec.width - MeasureText(valueTextBuffer, valueTextSize) - valueTextOffsetX),
             (int)(controlRec.y + controlRec.height / 2 - valueTextSize / 2), valueTextSize, labelColor);
    currentY += GUI_SLIDER_HEIGHT + GUI_VERTICAL_SPACING;
    if (fabsf(sim->perception_radius - old_perception) > EPSILON) Simulation_UpdateSquaredRadii(sim);

    DrawCenteredText("Separation Radius", currentY, view.width, view.x, GUI_LABEL_TEXT_SIZE, labelColor); currentY += GUI_LABEL_TEXT_SIZE + GUI_LABEL_CONTROL_SPACING;
    float old_separation = sim->separation_radius;
    controlRec = (Rectangle){ controlX, currentY, controlWidth, GUI_SLIDER_HEIGHT };
    GuiSliderBar(controlRec, NULL, NULL, &sim->separation_radius, 5.0f, 100.0f);
    snprintf(valueTextBuffer, 32, "%.1f", sim->separation_radius);
    DrawText(valueTextBuffer, (int)(controlRec.x + controlRec.width - MeasureText(valueTextBuffer, valueTextSize) - valueTextOffsetX),
             (int)(controlRec.y + controlRec.height / 2 - valueTextSize / 2), valueTextSize, labelColor);
    currentY += GUI_SLIDER_HEIGHT + GUI_VERTICAL_SPACING;
    if (fabsf(sim->separation_radius - old_separation) > EPSILON) Simulation_UpdateSquaredRadii(sim);

    DrawCenteredText("Edge Margin (3D)", currentY, view.width, view.x, GUI_LABEL_TEXT_SIZE, labelColor); currentY += GUI_LABEL_TEXT_SIZE + GUI_LABEL_CONTROL_SPACING;
    controlRec = (Rectangle){ controlX, currentY, controlWidth, GUI_SLIDER_HEIGHT };
    GuiSliderBar(controlRec, NULL, NULL, &sim->edge_margin, 10.0f, 150.0f);
    snprintf(valueTextBuffer, 32, "%.1f", sim->edge_margin);
    DrawText(valueTextBuffer, (int)(controlRec.x + controlRec.width - MeasureText(valueTextBuffer, valueTextSize) - valueTextOffsetX),
             (int)(controlRec.y + controlRec.height / 2 - valueTextSize / 2), valueTextSize, labelColor);
    currentY += GUI_SLIDER_HEIGHT + GUI_VERTICAL_SPACING;

    DrawCenteredText("Separation Weight", currentY, view.width, view.x, GUI_LABEL_TEXT_SIZE, labelColor); currentY += GUI_LABEL_TEXT_SIZE + GUI_LABEL_CONTROL_SPACING;
    controlRec = (Rectangle){ controlX, currentY, controlWidth, GUI_SLIDER_HEIGHT };
    GuiSliderBar(controlRec, NULL, NULL, &sim->separation_weight, 0.0f, 5.0f);
    snprintf(valueTextBuffer, 32, "%.2f", sim->separation_weight);
    DrawText(valueTextBuffer, (int)(controlRec.x + controlRec.width - MeasureText(valueTextBuffer, valueTextSize) - valueTextOffsetX),
             (int)(controlRec.y + controlRec.height / 2 - valueTextSize / 2), valueTextSize, labelColor);
    currentY += GUI_SLIDER_HEIGHT + GUI_VERTICAL_SPACING;

    DrawCenteredText("Alignment Weight", currentY, view.width, view.x, GUI_LABEL_TEXT_SIZE, labelColor); currentY += GUI_LABEL_TEXT_SIZE + GUI_LABEL_CONTROL_SPACING;
    controlRec = (Rectangle){ controlX, currentY, controlWidth, GUI_SLIDER_HEIGHT };
    GuiSliderBar(controlRec, NULL, NULL, &sim->alignment_weight, 0.0f, 5.0f);
    snprintf(valueTextBuffer, 32, "%.2f", sim->alignment_weight);
    DrawText(valueTextBuffer, (int)(controlRec.x + controlRec.width - MeasureText(valueTextBuffer, valueTextSize) - valueTextOffsetX),
             (int)(controlRec.y + controlRec.height / 2 - valueTextSize / 2), valueTextSize, labelColor);
    currentY += GUI_SLIDER_HEIGHT + GUI_VERTICAL_SPACING;

    DrawCenteredText("Cohesion Weight", currentY, view.width, view.x, GUI_LABEL_TEXT_SIZE, labelColor); currentY += GUI_LABEL_TEXT_SIZE + GUI_LABEL_CONTROL_SPACING;
    controlRec = (Rectangle){ controlX, currentY, controlWidth, GUI_SLIDER_HEIGHT };
    GuiSliderBar(controlRec, NULL, NULL, &sim->cohesion_weight, 0.0f, 5.0f);
    snprintf(valueTextBuffer, 32, "%.2f", sim->cohesion_weight);
    DrawText(valueTextBuffer, (int)(controlRec.x + controlRec.width - MeasureText(valueTextBuffer, valueTextSize) - valueTextOffsetX),
             (int)(controlRec.y + controlRec.height / 2 - valueTextSize / 2), valueTextSize, labelColor);
    currentY += GUI_SLIDER_HEIGHT + GUI_VERTICAL_SPACING;

    DrawCenteredText("Edge Avoid Weight", currentY, view.width, view.x, GUI_LABEL_TEXT_SIZE, labelColor); currentY += GUI_LABEL_TEXT_SIZE + GUI_LABEL_CONTROL_SPACING;
    controlRec = (Rectangle){ controlX, currentY, controlWidth, GUI_SLIDER_HEIGHT };
    GuiSliderBar(controlRec, NULL, NULL, &sim->edge_avoid_weight, 0.5f, 10.0f);
    snprintf(valueTextBuffer, 32, "%.2f", sim->edge_avoid_weight);
    DrawText(valueTextBuffer, (int)(controlRec.x + controlRec.width - MeasureText(valueTextBuffer, valueTextSize) - valueTextOffsetX),
             (int)(controlRec.y + controlRec.height / 2 - valueTextSize / 2), valueTextSize, labelColor);
    currentY += GUI_SLIDER_HEIGHT + GUI_VERTICAL_SPACING;

    // --- Rule Toggles Section --- //
    DrawCenteredText("--- Rule Toggles ---", currentY, view.width, view.x, GUI_LABEL_TEXT_SIZE + 2, headerColor);
    currentY += (GUI_LABEL_TEXT_SIZE + 2) + GUI_VERTICAL_SPACING;

    controlRec = (Rectangle){ controlX, currentY, GUI_CHECKBOX_SIZE, GUI_CHECKBOX_SIZE };
    GuiCheckBox(controlRec, " Separation Active", &sim->rule_separation_active);
    currentY += GUI_CHECKBOX_SIZE + GUI_LABEL_CONTROL_SPACING;

    controlRec = (Rectangle){ controlX, currentY, GUI_CHECKBOX_SIZE, GUI_CHECKBOX_SIZE };
    GuiCheckBox(controlRec, " Alignment Active", &sim->rule_alignment_active);
    currentY += GUI_CHECKBOX_SIZE + GUI_LABEL_CONTROL_SPACING;

    controlRec = (Rectangle){ controlX, currentY, GUI_CHECKBOX_SIZE, GUI_CHECKBOX_SIZE };
    GuiCheckBox(controlRec, " Cohesion Active", &sim->rule_cohesion_active);
    currentY += GUI_CHECKBOX_SIZE + GUI_LABEL_CONTROL_SPACING;

    controlRec = (Rectangle){ controlX, currentY, GUI_CHECKBOX_SIZE, GUI_CHECKBOX_SIZE };
    GuiCheckBox(controlRec, " Edge Avoidance Active", &sim->rule_edge_avoid_active);
    currentY += GUI_CHECKBOX_SIZE + GUI_VERTICAL_SPACING;

    // --- Simulation Options Section --- //
    DrawCenteredText("--- Simulation Options ---", currentY, view.width, view.x, GUI_LABEL_TEXT_SIZE + 2, headerColor);
    currentY += (GUI_LABEL_TEXT_SIZE + 2) + GUI_VERTICAL_SPACING;

    controlRec = (Rectangle){ controlX, currentY, controlWidth, GUI_BUTTON_HEIGHT };
    if (GuiButton(controlRec, "Reset Simulation")) {
        Simulation_Reset(sim);
    }
    currentY += GUI_BUTTON_HEIGHT + GUI_LABEL_CONTROL_SPACING;

    controlRec = (Rectangle){ controlX, currentY, controlWidth, GUI_BUTTON_HEIGHT };
    if (GuiButton(controlRec, sim->sim_paused ? "Resume Simulation (P)" : "Pause Simulation (P)")) {
        Simulation_TogglePause(sim);
    }
    currentY += GUI_BUTTON_HEIGHT + GUI_LABEL_CONTROL_SPACING;

    controlRec = (Rectangle){ controlX, currentY, GUI_CHECKBOX_SIZE, GUI_CHECKBOX_SIZE };
    GuiCheckBox(controlRec, " Auto-Rotate Camera", &sim->auto_rotate_camera);
    currentY += GUI_CHECKBOX_SIZE + GUI_VERTICAL_SPACING;

    // --- Application Section --- //
    DrawCenteredText("--- Application ---", currentY, view.width, view.x, GUI_LABEL_TEXT_SIZE + 2, headerColor);
    currentY += (GUI_LABEL_TEXT_SIZE + 2) + GUI_LABEL_CONTROL_SPACING;
    controlRec = (Rectangle){ controlX, currentY, controlWidth, GUI_BUTTON_HEIGHT };
    if (GuiButton(controlRec, "Exit Application")) {
        CloseWindow();
    }
    currentY += GUI_BUTTON_HEIGHT + GUI_VERTICAL_SPACING;
}
