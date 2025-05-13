// gui.h
#ifndef GUI_H
#define GUI_H

#include "raylib.h"
#include "simulation.h"

void GUI_SetupLayout(SimulationState *sim, Rectangle *panelRec, float *sliderStartX, float *sliderWidth);
void GUI_DrawTabsAndPanel(SimulationState *sim, const Rectangle *panelRec, float sliderStartX, float sliderWidth);
void GUI_DrawControlsTab(SimulationState *sim, Rectangle view);

#endif // GUI_H