// simulation.h
#ifndef SIMULATION_H
#define SIMULATION_H

#include "config.h" // For MAX_BOIDS
#include "raylib.h" // Includes raymath.h for Vector3 etc.
#include <stdbool.h>

// --- Boid Structure (3D) ---
typedef struct Boid {
    Vector3 position;     // 3D position
    Vector3 velocity;     // 3D velocity
    Vector3 acceleration; // 3D acceleration
    bool active;
    // Consider adding orientation (e.g., a Quaternion or a target/up vector)
    // if you want boids to visually orient themselves in 3D beyond just velocity direction.
    // For now, velocity direction can imply orientation for simple rendering.
} Boid;

// --- Simulation State Structure (3D) ---
typedef struct {
    // Boid data
    Boid *boids;
    int current_num_boids;
    // Vector3 last_separation_force[MAX_BOIDS]; // Removed Debug
    // Vector3 last_alignment_force[MAX_BOIDS];  // Removed Debug
    // Vector3 last_cohesion_force[MAX_BOIDS];   // Removed Debug
    // Vector3 last_edge_force[MAX_BOIDS];       // Removed Debug

    // Dimensions
    int screenWidth;        // For the 2D window
    int screenHeight;       // For the 2D window
    // int simulationWidth; // Replaced by cubeDimensions.x for simulation logic
    // Consider a Vector3 for simulation volume dimensions
    Vector3 cubeDimensions; // e.g., {width, height, depth} of the simulation area

    // Tunable Parameters (these are scalar, likely remain the same)
    float max_speed;
    float max_force;
    float perception_radius;
    float separation_radius;
    float edge_margin; // This will apply to all 6 faces of the cube

    // Precomputed Squared Radii (scalar, remain the same)
    float perceptionRadiusSq;
    float separationRadiusSq;

    // Behavior Weights (scalar, remain the same)
    float separation_weight;
    float alignment_weight;
    float cohesion_weight;
    float edge_avoid_weight;

    // --- Debugging State ---
    /*
    int selected_boid_index;
    bool show_perception;
    bool debug_draw_forces;
    bool debug_draw_neighbors;
    bool sim_paused;
    bool rule_separation_active;
    bool rule_alignment_active;
    bool rule_cohesion_active;
    bool rule_edge_avoid_active;
    bool debug_draw_grid;
    */
    // Moved required flags up
    bool sim_paused;
    bool rule_separation_active;
    bool rule_alignment_active;
    bool rule_cohesion_active;
    bool rule_edge_avoid_active;

    // GUI State (Simplified)
    // int active_gui_tab; // Removed
    // Vector2 debugScrollOffset; // Removed Debug
    // Rectangle debugScrollBounds; // Removed Debug
    // Rectangle debugScrollView;   // Removed Debug

    // --- Camera State ---
    bool auto_rotate_camera; // Added flag for toggling camera rotation

    // --- Spatial Grid Data (3D) ---
    float grid_cell_size;   // Scalar, remains the same
    int grid_dim_x;
    int grid_dim_y;
    int grid_dim_z;         // New dimension for the grid
    int* grid_boid_indices;
    int* grid_cell_starts;
    int* grid_cell_counts;

} SimulationState;

// --- Boid Logic Function Declarations (from original boid.h content, now 3D) ---
void InitializeBoid(Boid *boid, bool isActive, Vector3 cubeDimensions, float edgeMargin, float maxSpeed);
Vector3 GridSeparation(Boid *currentBoid, int currentBoidOriginalIndex, const SimulationState *sim);
Vector3 GridAlignment(Boid *currentBoid, int currentBoidOriginalIndex, const SimulationState *sim);
Vector3 GridCohesion(Boid *currentBoid, int currentBoidOriginalIndex, const SimulationState *sim);
Vector3 AvoidEdges(Boid *currentBoid, Vector3 cubeDimensions, float edgeMargin, float maxSpeed, float maxForce, float edgeWeightMultiplier);
void ApplyForce(Boid *boid, Vector3 force); // Now Vector3 force
void UpdateBoid(Boid *boid, float maxSpeed, Vector3 cubeDimensions); // Boundaries are now 3D
void DrawBoid(Boid *boid, Camera camera); // Drawing in 3D will require the camera

// --- Simulation Management Function Declarations ---
void Simulation_Initialize(SimulationState *sim);
void Simulation_Cleanup(SimulationState *sim);
void Simulation_UpdateGrid(SimulationState *sim);
void Simulation_Update(SimulationState *sim);
void Simulation_DrawBoids(SimulationState *sim, Camera camera); // Pass camera for 3D drawing
// void Simulation_DrawDebugInfo(SimulationState *sim, Camera camera); // Removed
void Simulation_Reset(SimulationState *sim);
void Simulation_SetBoidCount(SimulationState *sim, int newCount);
void Simulation_UpdateDimensions(SimulationState *sim); // Might need rethink for 3D sim volume vs 2D screen
void Simulation_UpdateSquaredRadii(SimulationState *sim);
// void Simulation_SelectBoidNear(SimulationState *sim, Camera camera); // Removed
void Simulation_TogglePause(SimulationState *sim);

#endif // SIMULATION_H
