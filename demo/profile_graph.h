namespace muli3
{

struct ProfileGraphEntry
{
    const char* name;
    uint32 color;
    const float* values;
};

void DrawProfileGraph(
    const char* label,
    Vec2 graphSize,
    int32 profileCapacity,
    int32 count,
    const ProfileGraphEntry* entries,
    int32 entryCount,
    bool showAxisLabels,
    bool showOverlay,
    bool showAverage
);

} // namespace muli3
