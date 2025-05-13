#include "simulation.h"
#include "config.h"
#include "utils.h"
#include "raymath.h"

#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdio.h>
#include <float.h>
#include <math.h>

#include <omp.h>

// Utility function to generate a random float within a given range.
float GetRandomFloat(float min, float max) {
    if (min >= max) return min; // Handle invalid range
    float scale = rand() / (float) RAND_MAX; // Get a random value between 0.0 and 1.0
    return min + scale * ( max - min ); // Scale and offset to the desired range
}

// Initializes a single boid with a random position and velocity.
// Position is within the cube, offset by the edge margin.
// Velocity is randomized but capped by maxSpeed.
void InitializeBoid(Boid *boid, bool isActive, Vector3 cubeDimensions, float edgeMargin, float maxSpeed) {
    // Calculate spawn area, ensuring it's not negative if margins are large
    float spawnWidth = (cubeDimensions.x > 2 * edgeMargin) ? (cubeDimensions.x - 2 * edgeMargin) : 1.0f;
    float spawnHeight = (cubeDimensions.y > 2 * edgeMargin) ? (cubeDimensions.y - 2 * edgeMargin) : 1.0f;
    float spawnDepth = (cubeDimensions.z > 2 * edgeMargin) ? (cubeDimensions.z - 2 * edgeMargin) : 1.0f;

    boid->position = (Vector3){
        edgeMargin + GetRandomFloat(0.0f, spawnWidth),
        edgeMargin + GetRandomFloat(0.0f, spawnHeight),
        edgeMargin + GetRandomFloat(0.0f, spawnDepth)
    };

    Vector3 randomDir = {
        GetRandomFloat(-1.0f, 1.0f),
        GetRandomFloat(-1.0f, 1.0f),
        GetRandomFloat(-1.0f, 1.0f)
    };
    if (Vector3LengthSqr(randomDir) < EPSILON) {
        randomDir = (Vector3){1.0f, 0.0f, 0.0f};
    }
    randomDir = Vector3Normalize(randomDir);

    float speed = GetRandomFloat(0.5f * maxSpeed, maxSpeed);
    if (speed < 0.5f) speed = 0.5f;
    boid->velocity = Vector3Scale(randomDir, speed);

    boid->acceleration = (Vector3){ 0.0f, 0.0f, 0.0f };
    boid->active = isActive;
}

// Applies a force vector to a boid's acceleration.
// This is used by the boid rules (separation, alignment, cohesion, etc.).
void ApplyForce(Boid *boid, Vector3 force) {
    boid->acceleration = Vector3Add(boid->acceleration, force);
}

// Updates a boid's velocity and position based on its current acceleration.
// Also enforces boundary conditions to keep the boid within the simulation cube.
void UpdateBoid(Boid *boid, float maxSpeed, Vector3 cubeDimensions) {
    // Update velocity with acceleration, then limit to max speed
    boid->velocity = Vector3Add(boid->velocity, boid->acceleration);
    boid->velocity = Vector3Limit(boid->velocity, maxSpeed);
    boid->position = Vector3Add(boid->position, boid->velocity);
    boid->acceleration = (Vector3){ 0.0f, 0.0f, 0.0f }; // Reset acceleration for the next frame

    // Simple clamping to keep boids within the cube boundaries.
    // A small buffer is used to prevent issues at the exact edge.
    float buffer = 0.1f;

    if (boid->position.x < buffer) {
        boid->position.x = buffer;
    } else if (boid->position.x > cubeDimensions.x - buffer) {
        boid->position.x = cubeDimensions.x - buffer;
    }

    if (boid->position.y < buffer) {
        boid->position.y = buffer;
    } else if (boid->position.y > cubeDimensions.y - buffer) {
        boid->position.y = cubeDimensions.y - buffer;
    }

    if (boid->position.z < buffer) {
        boid->position.z = buffer;
    } else if (boid->position.z > cubeDimensions.z - buffer) {
        boid->position.z = cubeDimensions.z - buffer;
    }
}

// Draws a single boid as an oriented cone.
// The cone points in the direction of the boid's velocity.
// If the boid is nearly stationary, it's drawn as a small sphere.
void DrawBoid(Boid *boid, Camera camera) {
    if (!boid->active) return; // Don't draw inactive boids

    float coneRadius = BOID_SIZE;
    float coneHeight = BOID_SIZE * 2.5f; // Make the cone elongated

    // If boid has negligible velocity, draw as a simple sphere (e.g., when paused or just spawned)
    if (Vector3LengthSqr(boid->velocity) < EPSILON * EPSILON) {
        DrawSphere(boid->position, coneRadius * 0.8f, DARKGRAY);
        return;
    }

    Vector3 normalizedVelocity = Vector3Normalize(boid->velocity);

    Vector3 coneBaseCenter = Vector3Subtract(boid->position, Vector3Scale(normalizedVelocity, coneHeight * 0.33f));
    Vector3 coneTip = Vector3Add(boid->position, Vector3Scale(normalizedVelocity, coneHeight * 0.67f));

    DrawCylinderEx(coneBaseCenter, coneTip, coneRadius, 0.0f, 12, ORANGE);
}

// Function pointer type for processing a neighboring boid found via the grid.
// Used by the rule functions (Separation, Alignment, Cohesion).
typedef void (*ProcessNeighborFunc)(Boid* currentBoid, Boid* otherBoid, float distSq, const SimulationState* sim, void* ruleData);

// Iterates over boids in the neighboring grid cells of a given boid.
// For each neighbor found, it calls the provided `processFunc`.
// This is the core of the spatial grid optimization for neighbor searching.
void ForEachNeighborInGrid(Boid *currentBoid, int currentBoidOriginalIndex, const SimulationState *sim,
                           ProcessNeighborFunc processFunc, void* ruleData) {
    // Basic checks to ensure grid is valid and there are boids to process
    if (!sim->grid_cell_counts || !sim->grid_boid_indices || !sim->grid_cell_starts ||
        sim->current_num_boids == 0 ||
        sim->grid_dim_x == 0 || sim->grid_dim_y == 0 || sim->grid_dim_z == 0) {
        return;
    }

    // Determine the grid cell coordinates (cx, cy, cz) for the current boid
    float boid_x = fmaxf(0.0f, fminf(currentBoid->position.x, sim->cubeDimensions.x - EPSILON));
    float boid_y = fmaxf(0.0f, fminf(currentBoid->position.y, sim->cubeDimensions.y - EPSILON));
    float boid_z = fmaxf(0.0f, fminf(currentBoid->position.z, sim->cubeDimensions.z - EPSILON));

    int cx = (int)(boid_x / sim->grid_cell_size);
    int cy = (int)(boid_y / sim->grid_cell_size);
    int cz = (int)(boid_z / sim->grid_cell_size);

    if (cx < 0) cx = 0; else if (cx >= sim->grid_dim_x) cx = sim->grid_dim_x - 1;
    if (cy < 0) cy = 0; else if (cy >= sim->grid_dim_y) cy = sim->grid_dim_y - 1;
    if (cz < 0) cz = 0; else if (cz >= sim->grid_dim_z) cz = sim->grid_dim_z - 1;

    // Iterate over the 3x3x3 block of cells around the boid's current cell (including its own)
    for (int NBR_cz = cz - 1; NBR_cz <= cz + 1; NBR_cz++) {
        for (int NBR_cy = cy - 1; NBR_cy <= cy + 1; NBR_cy++) {
            for (int NBR_cx = cx - 1; NBR_cx <= cx + 1; NBR_cx++) {
                if (NBR_cx < 0 || NBR_cx >= sim->grid_dim_x ||
                    NBR_cy < 0 || NBR_cy >= sim->grid_dim_y ||
                    NBR_cz < 0 || NBR_cz >= sim->grid_dim_z) {
                    continue;
                }
                int cell_flat_index = NBR_cx + (NBR_cy * sim->grid_dim_x) + (NBR_cz * sim->grid_dim_x * sim->grid_dim_y);
                if (cell_flat_index < 0 || cell_flat_index >= (sim->grid_dim_x * sim->grid_dim_y * sim->grid_dim_z)) {
                    continue;
                }
                int start_index_in_grid = sim->grid_cell_starts[cell_flat_index];
                int num_boids_in_cell = sim->grid_cell_counts[cell_flat_index];
                for (int k = 0; k < num_boids_in_cell; k++) {
                    int other_boid_original_index = sim->grid_boid_indices[start_index_in_grid + k];
                    if (other_boid_original_index == currentBoidOriginalIndex) continue;
                    if (other_boid_original_index < 0 || other_boid_original_index >= sim->current_num_boids) continue; // Bounds check
                    Boid *other = &sim->boids[other_boid_original_index];
                    if (!other->active) continue;

                    // Calculate squared distance and call the processing function
                    float distSq = Vector3DistanceSqr(currentBoid->position, other->position);
                    processFunc(currentBoid, other, distSq, sim, ruleData);
                }
            }
        }
    }
}

// --- Boid Behavior Rules (using the spatial grid) --- //

// Data structure for accumulating separation force components.
typedef struct { Vector3 steer; int count; } SeparationData;

// Process function for the separation rule.
// If a neighbor is within the separation radius, a repulsive force is calculated.
void GridSeparation_Process(Boid* currentBoid, Boid* otherBoid, float distSq, const SimulationState* sim, void* ruleData) {
    SeparationData* data = (SeparationData*)ruleData;
    if (distSq > EPSILON && distSq < sim->separationRadiusSq) {
        Vector3 diff = Vector3Subtract(currentBoid->position, otherBoid->position);
        float weight = 1.0f / distSq;
        data->steer = Vector3Add(data->steer, Vector3Scale(diff, weight));
        data->count++;
    }
}

// Calculates the separation steering force for a boid.
// This force pushes the boid away from nearby flockmates to avoid crowding.
Vector3 GridSeparation(Boid *currentBoid, int currentBoidOriginalIndex, const SimulationState *sim) {
    SeparationData data = {{0},0}; // Initialize steer vector and count
    ForEachNeighborInGrid(currentBoid, currentBoidOriginalIndex, sim, GridSeparation_Process, &data);
    Vector3 final_steer = {0};
    if (data.count > 0) { // If any neighbors were close enough
        if (Vector3LengthSqr(data.steer) > EPSILON) {
            // Average the steering vector and scale it to max_speed
            data.steer = Vector3Normalize(data.steer);
            data.steer = Vector3Scale(data.steer, sim->max_speed);
            final_steer = Vector3Subtract(data.steer, currentBoid->velocity);
            final_steer = Vector3Limit(final_steer, sim->max_force);
        }
    }
    return final_steer;
}

// Data structure for accumulating alignment force components.
typedef struct { Vector3 sum_velocity; int count; } AlignmentData;

// Process function for the alignment rule.
// Sums velocities of neighbors within the perception radius.
void GridAlignment_Process(Boid* currentBoid, Boid* otherBoid, float distSq, const SimulationState* sim, void* ruleData) {
    AlignmentData* data = (AlignmentData*)ruleData;
    if (distSq > EPSILON && distSq < sim->perceptionRadiusSq) {
        data->sum_velocity = Vector3Add(data->sum_velocity, otherBoid->velocity);
        data->count++;
    }
}

// Calculates the alignment steering force for a boid.
// This force encourages the boid to align its velocity with that of its nearby flockmates.
Vector3 GridAlignment(Boid *currentBoid, int currentBoidOriginalIndex, const SimulationState *sim) {
    AlignmentData data = {{0},0};
    ForEachNeighborInGrid(currentBoid, currentBoidOriginalIndex, sim, GridAlignment_Process, &data);
    Vector3 steer = {0};
    if (data.count > 0) {
        // Average the sum of velocities
        data.sum_velocity = Vector3Scale(data.sum_velocity, 1.0f / (float)data.count);
        if (Vector3LengthSqr(data.sum_velocity) > EPSILON) {
            // Normalize and scale to max_speed to get desired velocity
            data.sum_velocity = Vector3Normalize(data.sum_velocity);
            data.sum_velocity = Vector3Scale(data.sum_velocity, sim->max_speed);
            steer = Vector3Subtract(data.sum_velocity, currentBoid->velocity);
            steer = Vector3Limit(steer, sim->max_force);
        }
    }
    return steer;
}

// Data structure for accumulating cohesion force components.
typedef struct { Vector3 sum_position; int count; } CohesionData;

// Process function for the cohesion rule.
// Sums positions of neighbors within the perception radius.
void GridCohesion_Process(Boid* currentBoid, Boid* otherBoid, float distSq, const SimulationState* sim, void* ruleData) {
    CohesionData* data = (CohesionData*)ruleData;
    if (distSq > EPSILON && distSq < sim->perceptionRadiusSq) {
        data->sum_position = Vector3Add(data->sum_position, otherBoid->position);
        data->count++;
    }
}

// Calculates the cohesion steering force for a boid.
// This force encourages the boid to move towards the center of mass of its nearby flockmates.
Vector3 GridCohesion(Boid *currentBoid, int currentBoidOriginalIndex, const SimulationState *sim) {
    CohesionData data = {{0},0};
    ForEachNeighborInGrid(currentBoid, currentBoidOriginalIndex, sim, GridCohesion_Process, &data);
    Vector3 steer = {0};
    if (data.count > 0) {
        // Calculate average position (center of mass)
        data.sum_position = Vector3Scale(data.sum_position, 1.0f / (float)data.count);
        // Calculate desired velocity vector towards the center of mass
        Vector3 desired = Vector3Subtract(data.sum_position, currentBoid->position);
        if (Vector3LengthSqr(desired) > EPSILON) {
            // Normalize and scale to max_speed
            desired = Vector3Normalize(desired);
            desired = Vector3Scale(desired, sim->max_speed);
            steer = Vector3Subtract(desired, currentBoid->velocity);
            steer = Vector3Limit(steer, sim->max_force);
        }
    }
    return steer;
}

// Calculates a steering force to keep boids within the simulation cube.
// It applies a force away from any face of the cube if the boid gets too close (within edgeMargin).
Vector3 AvoidEdges(Boid *currentBoid, Vector3 cubeDimensions, float edgeMargin, float maxSpeed, float maxForce, float edgeWeightMultiplier) {
    Vector3 steer = {0.0f, 0.0f, 0.0f};
    Vector3 desiredVelocity = {0.0f, 0.0f, 0.0f};
    bool applyForce = false;

    // Check X faces
    if (currentBoid->position.x < edgeMargin) {
        desiredVelocity.x = maxSpeed; // Steer right
        applyForce = true;
    } else if (currentBoid->position.x > cubeDimensions.x - edgeMargin) {
        desiredVelocity.x = -maxSpeed; // Steer left
        applyForce = true;
    }

    // Check Y faces
    if (currentBoid->position.y < edgeMargin) {
        desiredVelocity.y = maxSpeed; // Steer up
        applyForce = true;
    } else if (currentBoid->position.y > cubeDimensions.y - edgeMargin) {
        desiredVelocity.y = -maxSpeed; // Steer down
        applyForce = true;
    }

    // Check Z faces
    if (currentBoid->position.z < edgeMargin) {
        desiredVelocity.z = maxSpeed; // Steer forward (towards positive Z, assuming camera looks from negative Z)
        applyForce = true;
    } else if (currentBoid->position.z > cubeDimensions.z - edgeMargin) {
        desiredVelocity.z = -maxSpeed; // Steer backward
        applyForce = true;
    }

    if (applyForce) {
        // If multiple edges are hit, desiredVelocity will have components for each.
        // This is a simple additive approach. More sophisticated might prioritize or blend.
        if (Vector3LengthSqr(desiredVelocity) > EPSILON) { // Check if any desired velocity was set
            desiredVelocity = Vector3Normalize(desiredVelocity);
            desiredVelocity = Vector3Scale(desiredVelocity, maxSpeed);
            steer = Vector3Subtract(desiredVelocity, currentBoid->velocity);
            steer = Vector3Limit(steer, maxForce * edgeWeightMultiplier);
        }
    }
    return steer;
}

// --- Spatial Grid Management --- //

// Allocates (or reallocates) memory for the spatial grid arrays.
// This is called during initialization and if grid dimensions change (e.g., perception radius changes).
void AllocateGridMemory(SimulationState *sim) {
    // Free existing memory if any (for reallocation)
    if (sim->grid_boid_indices) free(sim->grid_boid_indices);
    if (sim->grid_cell_starts) free(sim->grid_cell_starts);
    if (sim->grid_cell_counts) free(sim->grid_cell_counts);

    int num_cells = sim->grid_dim_x * sim->grid_dim_y * sim->grid_dim_z;
    if (num_cells <= 0 || sim->grid_dim_x <= 0 || sim->grid_dim_y <= 0 || sim->grid_dim_z <= 0) {
        sim->grid_boid_indices = NULL;
        sim->grid_cell_starts = NULL;
        sim->grid_cell_counts = NULL;
        TraceLog(LOG_WARNING, "AllocateGridMemory: Invalid grid dimensions, memory not allocated.");
        return;
    }

    sim->grid_boid_indices = (int*)malloc(MAX_BOIDS * sizeof(int));
    sim->grid_cell_starts = (int*)malloc(num_cells * sizeof(int));
    sim->grid_cell_counts = (int*)malloc(num_cells * sizeof(int));

    if (!sim->grid_boid_indices || !sim->grid_cell_starts || !sim->grid_cell_counts) {
        TraceLog(LOG_ERROR, "Failed to allocate grid memory!");
        if (sim->grid_boid_indices) { free(sim->grid_boid_indices); sim->grid_boid_indices = NULL; }
        if (sim->grid_cell_starts) { free(sim->grid_cell_starts); sim->grid_cell_starts = NULL; }
        if (sim->grid_cell_counts) { free(sim->grid_cell_counts); sim->grid_cell_counts = NULL; }
    } else {
        memset(sim->grid_cell_counts, 0, num_cells * sizeof(int));
    }
}

// Initializes the entire simulation state.
// Sets default parameters, allocates memory for boids and the grid.
void Simulation_Initialize(SimulationState *sim) {
    srand((unsigned int) time(NULL)); // Seed random number generator

    // Set initial simulation parameters from config.h or defaults
    sim->cubeDimensions = (Vector3){400.0f, 300.0f, 300.0f};
    sim->current_num_boids = 100;
    sim->max_speed = 3.0f;
    sim->max_force = 0.08f;
    sim->perception_radius = 50.0f;
    sim->separation_radius = 25.0f;
    sim->edge_margin = 30.0f;
    sim->separation_weight = 1.5f;
    sim->alignment_weight = 1.0f;
    sim->cohesion_weight = 1.0f;
    sim->edge_avoid_weight = 2.0f;

    sim->sim_paused = false;
    sim->rule_separation_active = true;
    sim->rule_alignment_active = true;
    sim->rule_cohesion_active = true;
    sim->rule_edge_avoid_active = true;

    sim->auto_rotate_camera = true; // Start with camera auto-rotation enabled

    // Initialize grid parameters (cell size based on perception radius)
    sim->grid_cell_size = sim->perception_radius;
    if (sim->grid_cell_size < 1.0f) sim->grid_cell_size = 1.0f;
    sim->grid_dim_x = 0; sim->grid_dim_y = 0; sim->grid_dim_z = 0;
    sim->grid_boid_indices = NULL; sim->grid_cell_starts = NULL; sim->grid_cell_counts = NULL;

    sim->screenWidth = GetScreenWidth();
    sim->screenHeight = GetScreenHeight();
    if (sim->screenWidth == 0) sim->screenWidth = 2560;
    if (sim->screenHeight == 0) sim->screenHeight = 1440;

    // Allocate memory for the boids array
    sim->boids = (Boid*)malloc(MAX_BOIDS * sizeof(Boid));
    if (sim->boids == NULL) {
        TraceLog(LOG_FATAL, "Failed to allocate memory for boids array!");
        exit(1);
    }
    for (int i = 0; i < MAX_BOIDS; i++) {
        InitializeBoid(&sim->boids[i], (i < sim->current_num_boids), sim->cubeDimensions,
                       sim->edge_margin, sim->max_speed);
    }

    Simulation_UpdateDimensions(sim);
    Simulation_UpdateSquaredRadii(sim); // Precompute squared radii for performance
}

// Frees all dynamically allocated memory used by the simulation.
void Simulation_Cleanup(SimulationState *sim) {
    if (sim->boids) free(sim->boids);
    if (sim->grid_boid_indices) free(sim->grid_boid_indices);
    if (sim->grid_cell_starts) free(sim->grid_cell_starts);
    if (sim->grid_cell_counts) free(sim->grid_cell_counts);
    sim->grid_boid_indices = NULL; sim->grid_cell_starts = NULL; sim->grid_cell_counts = NULL;
}

// Updates grid dimensions and reallocates grid memory if necessary.
// This is typically called when perception radius (and thus grid_cell_size) changes.
void Simulation_UpdateDimensions(SimulationState *sim) {
    // Ensure grid cell size and cube dimensions are valid before calculating new grid dims
    if (sim->grid_cell_size > 0.0f &&
        sim->cubeDimensions.x > 0 && sim->cubeDimensions.y > 0 && sim->cubeDimensions.z > 0) {
        int old_dim_x = sim->grid_dim_x; int old_dim_y = sim->grid_dim_y; int old_dim_z = sim->grid_dim_z;
        sim->grid_dim_x = (int)ceilf(sim->cubeDimensions.x / sim->grid_cell_size);
        sim->grid_dim_y = (int)ceilf(sim->cubeDimensions.y / sim->grid_cell_size);
        sim->grid_dim_z = (int)ceilf(sim->cubeDimensions.z / sim->grid_cell_size);
        if (sim->grid_dim_x <= 0) sim->grid_dim_x = 1;
        if (sim->grid_dim_y <= 0) sim->grid_dim_y = 1;
        if (sim->grid_dim_z <= 0) sim->grid_dim_z = 1;
        if (sim->grid_dim_x != old_dim_x || sim->grid_dim_y != old_dim_y || sim->grid_dim_z != old_dim_z ||
            sim->grid_boid_indices == NULL) {
            AllocateGridMemory(sim);
        }
    } else {
        TraceLog(LOG_WARNING, "Simulation_UpdateDimensions: Invalid grid_cell_size or cubeDimensions, grid not updated.");
    }
}

// Updates the spatial grid with current boid positions.
// This involves three passes:
// 1. Count boids in each cell.
// 2. Calculate start indices for each cell in the flattened grid_boid_indices array.
// 3. Populate grid_boid_indices with the actual boid indices for each cell.
void Simulation_UpdateGrid(SimulationState *sim) {
    // Safety checks for grid and boid data
    if (!sim->grid_cell_counts || !sim->grid_cell_starts || !sim->grid_boid_indices ||
         sim->grid_dim_x == 0 || sim->grid_dim_y == 0 || sim->grid_dim_z == 0) {
        TraceLog(LOG_DEBUG, "Grid not initialized, skipping grid update.");
        return;
    }
    if (sim->current_num_boids == 0) {
        if (sim->grid_cell_counts) {
             int num_cells_to_clear = sim->grid_dim_x * sim->grid_dim_y * sim->grid_dim_z;
             if (num_cells_to_clear > 0) {
                memset(sim->grid_cell_counts, 0, num_cells_to_clear * sizeof(int));
             }
        }
        return;
    }

    // Pass 1: Count boids in each cell (parallelized with OpenMP)
    int num_cells = sim->grid_dim_x * sim->grid_dim_y * sim->grid_dim_z;
    memset(sim->grid_cell_counts, 0, num_cells * sizeof(int));
    int i_p1;
    #pragma omp parallel for default(none) shared(sim) private(i_p1) schedule(dynamic)
    for (i_p1 = 0; i_p1 < sim->current_num_boids; i_p1++) {
        if (sim->boids[i_p1].active) {
            float boid_x = fmaxf(0.0f, fminf(sim->boids[i_p1].position.x, sim->cubeDimensions.x - EPSILON));
            float boid_y = fmaxf(0.0f, fminf(sim->boids[i_p1].position.y, sim->cubeDimensions.y - EPSILON));
            float boid_z = fmaxf(0.0f, fminf(sim->boids[i_p1].position.z, sim->cubeDimensions.z - EPSILON));
            int cx = (int)(boid_x / sim->grid_cell_size);
            int cy = (int)(boid_y / sim->grid_cell_size);
            int cz = (int)(boid_z / sim->grid_cell_size);
            if (cx < 0) cx = 0; else if (cx >= sim->grid_dim_x) cx = sim->grid_dim_x - 1;
            if (cy < 0) cy = 0; else if (cy >= sim->grid_dim_y) cy = sim->grid_dim_y - 1;
            if (cz < 0) cz = 0; else if (cz >= sim->grid_dim_z) cz = sim->grid_dim_z - 1;
            int cell_flat_index = cx + (cy * sim->grid_dim_x) + (cz * sim->grid_dim_x * sim->grid_dim_y);
            #pragma omp atomic update
            sim->grid_cell_counts[cell_flat_index]++;
        }
    }

    // Pass 2: Calculate start indices for each cell in the global boid index list
    sim->grid_cell_starts[0] = 0;
    for (int k = 1; k < num_cells; k++) {
        sim->grid_cell_starts[k] = sim->grid_cell_starts[k-1] + sim->grid_cell_counts[k-1];
    }

    // Pass 3: Populate the grid_boid_indices array (parallelized with OpenMP)
    // This array stores, for each cell, the original indices of boids within it.
    int* cell_offsets = (int*)calloc(num_cells, sizeof(int)); // Temporary array to track current write offset within each cell's section
    if (!cell_offsets) { TraceLog(LOG_ERROR, "Failed to allocate cell_offsets for grid update Pass 3."); return; }
    int i_p3;
    #pragma omp parallel for default(none) shared(sim, cell_offsets) private(i_p3) schedule(dynamic)
    for (i_p3 = 0; i_p3 < sim->current_num_boids; i_p3++) {
        if (sim->boids[i_p3].active) {
            float boid_x = fmaxf(0.0f, fminf(sim->boids[i_p3].position.x, sim->cubeDimensions.x - EPSILON));
            float boid_y = fmaxf(0.0f, fminf(sim->boids[i_p3].position.y, sim->cubeDimensions.y - EPSILON));
            float boid_z = fmaxf(0.0f, fminf(sim->boids[i_p3].position.z, sim->cubeDimensions.z - EPSILON));
            int cx = (int)(boid_x / sim->grid_cell_size);
            int cy = (int)(boid_y / sim->grid_cell_size);
            int cz = (int)(boid_z / sim->grid_cell_size);
            if (cx < 0) cx = 0; else if (cx >= sim->grid_dim_x) cx = sim->grid_dim_x - 1;
            if (cy < 0) cy = 0; else if (cy >= sim->grid_dim_y) cy = sim->grid_dim_y - 1;
            if (cz < 0) cz = 0; else if (cz >= sim->grid_dim_z) cz = sim->grid_dim_z - 1;
            int cell_flat_index = cx + (cy * sim->grid_dim_x) + (cz * sim->grid_dim_x * sim->grid_dim_y);
            int local_offset_in_cell;
            #pragma omp atomic capture
            local_offset_in_cell = cell_offsets[cell_flat_index]++;
            int write_index_in_grid = sim->grid_cell_starts[cell_flat_index] + local_offset_in_cell;
            if (write_index_in_grid < MAX_BOIDS && local_offset_in_cell < sim->grid_cell_counts[cell_flat_index]) {
                sim->grid_boid_indices[write_index_in_grid] = i_p3;
             } else {
                 TraceLog(LOG_WARNING, "Grid write index out of bounds (boid %d, cell %d, offset %d, write_idx %d, cell_count %d, grid_start %d)",
                             i_p3, cell_flat_index, local_offset_in_cell, write_index_in_grid, sim->grid_cell_counts[cell_flat_index], sim->grid_cell_starts[cell_flat_index]);
             }
        }
    }
    free(cell_offsets);
}

// Main update function for the simulation, called each frame.
// If not paused, it updates the grid, calculates boid forces, and updates boid states.
void Simulation_Update(SimulationState *sim) {
    if (sim->sim_paused) return; // Do nothing if paused

    // 1. Update the spatial grid based on current boid positions
    Simulation_UpdateGrid(sim);

    // 2. Calculate forces and update boids in parallel (using OpenMP)
    int i_force; // Declare loop variable outside for OpenMP compatibility
#pragma omp parallel for shared(sim) private(i_force) schedule(dynamic)
    for (i_force = 0; i_force < sim->current_num_boids; ++i_force) {
        if (!sim->boids[i_force].active) continue;

        Boid *currentBoid = &sim->boids[i_force];

        // Calculate forces using the grid
        Vector3 separation = {0}, alignment = {0}, cohesion = {0}, edgeAvoid = {0};

        if (sim->rule_separation_active) {
            separation = GridSeparation(currentBoid, i_force, sim);
            separation = Vector3Scale(separation, sim->separation_weight);
            ApplyForce(currentBoid, separation);
        }

        if (sim->rule_alignment_active) {
            alignment = GridAlignment(currentBoid, i_force, sim);
            alignment = Vector3Scale(alignment, sim->alignment_weight);
            ApplyForce(currentBoid, alignment);
        }

        if (sim->rule_cohesion_active) {
            cohesion = GridCohesion(currentBoid, i_force, sim);
            cohesion = Vector3Scale(cohesion, sim->cohesion_weight);
            ApplyForce(currentBoid, cohesion);
        }

        if (sim->rule_edge_avoid_active) {
            edgeAvoid = AvoidEdges(currentBoid, sim->cubeDimensions, sim->edge_margin, sim->max_speed, sim->max_force, sim->edge_avoid_weight);
            ApplyForce(currentBoid, edgeAvoid);
        }
    }

    // 3. Update boid positions and velocities (can also be parallelized with OpenMP)
    int i_update; // Declare loop variable outside
#pragma omp parallel for shared(sim) private(i_update) schedule(dynamic)
    for (i_update = 0; i_update < sim->current_num_boids; ++i_update) {
        if (sim->boids[i_update].active) {
            UpdateBoid(&sim->boids[i_update], sim->max_speed, sim->cubeDimensions);
        }
    }
}

// Precomputes squared versions of perception and separation radii.
// Used for distance comparisons to avoid costly square root operations.
// Also updates grid dimensions if perception radius (and thus cell size) changes.
void Simulation_UpdateSquaredRadii(SimulationState *sim) {
    sim->perceptionRadiusSq = sim->perception_radius * sim->perception_radius;
    sim->separationRadiusSq = sim->separation_radius * sim->separation_radius;
    float old_cell_size = sim->grid_cell_size;
    sim->grid_cell_size = sim->perception_radius;
    if (sim->grid_cell_size < 1.0f) sim->grid_cell_size = 1.0f;
    if (fabsf(sim->grid_cell_size - old_cell_size) > EPSILON) {
        Simulation_UpdateDimensions(sim);
    }
}

// Draws all active boids in the simulation.
void Simulation_DrawBoids(SimulationState *sim, Camera camera) {
    if (!sim) return;
    for (int i = 0; i < sim->current_num_boids; i++) {
        if (sim->boids[i].active) {
            DrawBoid(&sim->boids[i], camera);
        }
    }
}

// Toggles the pause state of the simulation.
void Simulation_TogglePause(SimulationState *sim) {
    if (!sim) return; sim->sim_paused = !sim->sim_paused;
    if (sim->sim_paused) TraceLog(LOG_INFO, "Simulation Paused.");
    else TraceLog(LOG_INFO, "Simulation Resumed.");
}

// Resets the simulation to its initial state.
// Reinitializes boids, updates the grid, and resets pause state.
void Simulation_Reset(SimulationState *sim) {
    if (!sim) return;
    // Re-initialize all boids (active ones based on current_num_boids)
    for (int i = 0; i < MAX_BOIDS; i++) {
        bool isActive = (i < sim->current_num_boids);
        InitializeBoid(&sim->boids[i], isActive, sim->cubeDimensions, sim->edge_margin, sim->max_speed);
    }
    Simulation_UpdateGrid(sim);
    TraceLog(LOG_INFO, "Simulation Reset (3D).");
}

// Sets the number of active boids in the simulation.
// Activates or deactivates boids as needed.
void Simulation_SetBoidCount(SimulationState *sim, int newCount) {
    if (!sim) return; int oldBoidCount = sim->current_num_boids;
    // Clamp newCount to valid range (0 to MAX_BOIDS)
    if (newCount < 0) newCount = 0; if (newCount > MAX_BOIDS) newCount = MAX_BOIDS;
    sim->current_num_boids = newCount;

    // If increasing boid count, initialize and activate new boids
    if (newCount > oldBoidCount) {
        for (int i = oldBoidCount; i < newCount; i++) {
             InitializeBoid(&sim->boids[i], true, sim->cubeDimensions, sim->edge_margin, sim->max_speed);
        }
    } else if (newCount < oldBoidCount) { // If decreasing, deactivate surplus boids
        for (int i = newCount; i < oldBoidCount; i++) { sim->boids[i].active = false; }
    }
    TraceLog(LOG_INFO, "Boid count set to %d", sim->current_num_boids);
}
