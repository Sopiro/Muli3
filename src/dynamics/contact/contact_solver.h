#pragma once

#include "muli3/common.h"

namespace muli3
{

struct BlockContactArray;
struct SolverSet;
struct ContactState;
struct JointState;
struct ScalarContactConstraint;
struct Timestep;

void PrepareContactBlock(BlockContactArray* contacts, SolverSet* solverSets, int32 block);
void WarmStartContactBlock(BlockContactArray* contacts, int32 block);
void SolveContactVelocityBlock(BlockContactArray* contacts, int32 block);
void SolveContactPositionBlock(BlockContactArray* contacts, int32 block);

void PrepareContactScalar(ContactState* state, ScalarContactConstraint* constraint);
void WarmStartContactScalar(ContactState* state, ScalarContactConstraint* constraint);
void SolveContactVelocityScalar(ContactState* state, ScalarContactConstraint* constraint);
void SolveContactPositionScalar(ContactState* state, ScalarContactConstraint* constraint);

void PrepareJoint(JointState* joint, const Timestep& step);
void WarmStartJoint(JointState* joint);
void SolveJointVelocity(JointState* joint, const Timestep& step);

} // namespace muli3