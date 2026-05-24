namespace muli3
{

enum ProfileValue
{
    profile_broad_phase,
    profile_narrow_phase,
    profile_deferred_destroy,
    profile_step_other,
    profile_build_islands,
    profile_solve_islands,
    profile_sync_transforms,
    profile_finalize,
};

struct ProfileGraphEntry
{
    const char* name;
    uint32 color;
    ProfileValue value;
};

void DrawProfileGraph(
    const char* label,
    const WorldProfile* profiles,
    Vec2 graphSize,
    int32 profileCapacity,
    uint64 profileReadIndex,
    int32 count,
    const ProfileGraphEntry* entries,
    int32 entryCount,
    float maxRange,
    bool showOverlay,
    bool showAverage
);

} // namespace muli3
