#pragma once

#include "solver_states.h"

namespace muli3
{

void PrepareContactBlock(BlockContactArray* contacts, SolverSet* solverSets, int32 block);
void WarmStartContactBlock(BlockContactArray* contacts, int32 block);
void SolveContactVelocityBlock(BlockContactArray* contacts, int32 block);
uint32 SolveContactPositionBlock(BlockContactArray* contacts, int32 block);

} // namespace muli3
