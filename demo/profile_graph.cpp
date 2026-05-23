#include "profile_graph.h"

namespace muli3
{

static ImU32 ToImColor(uint32 c)
{
    return IM_COL32((c >> 16) & 0xff, (c >> 8) & 0xff, c & 0xff, 255);
}

static float GetProfileValue(const WorldProfile& profile, ProfileValue value)
{
    switch (value)
    {
    case profile_broad_phase:
        return profile.broad_phase;
    case profile_narrow_phase:
        return profile.narrow_phase;
    case profile_deferred_destroy:
        return profile.deferred_destroy;
    case profile_build_islands:
        return profile.build_islands;
    case profile_solve_islands:
        return profile.solve_islands;
    case profile_sync_transforms:
        return profile.sync_transforms;
    case profile_clear_island_flags:
        return profile.clear_island_flags;
    case profile_step_other:
        return (std::max)(0.0f,
                          profile.step - profile.broad_phase - profile.narrow_phase - profile.solve - profile.deferred_destroy);
    default:
        return 0.0f;
    }
}

static float GetProfileTotal(const WorldProfile& profile, const ProfileGraphEntry* entries, int32 entryCount)
{
    float total = 0.0f;

    for (int32 i = 0; i < entryCount; ++i)
    {
        total += GetProfileValue(profile, entries[i].value);
    }

    return total;
}

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
)
{
    if (count <= 0)
    {
        ImGui::TextUnformatted(label);
        return;
    }

    float maxTotal = 0.0f;
    float minTotal = 0.0f;
    float sumTotal = 0.0f;
    float nowTotal = 0.0f;

    for (int32 i = 0; i < count; ++i)
    {
        int32 index = (int32)((profileReadIndex + i) & (profileCapacity - 1));
        float total = GetProfileTotal(profiles[index], entries, entryCount);

        if (i == 0)
        {
            minTotal = total;
            maxTotal = total;
        }
        else
        {
            minTotal = (std::min)(minTotal, total);
            maxTotal = (std::max)(maxTotal, total);
        }

        sumTotal += total;
        nowTotal = total;
    }

    float minValue = 0.0f;
    float maxValue = (std::max)(maxTotal, 0.1f);
    if (maxRange > 0)
    {
        maxValue = (std::min)(maxValue, maxRange);
    }
    maxValue = (std::max)(maxValue, 0.001f);

    if (strlen(label) > 0)
    {
        ImGui::Text("%s", label);
    }

    ImDrawList* drawList = ImGui::GetWindowDrawList();
    ImVec2 plotSize{ graphSize.x, graphSize.y };
    ImVec2 legendSize{ 240.0f, plotSize.y };
    ImVec2 spacing{ 14.0f, 0.0f };
    ImVec2 canvasSize{ plotSize.x + spacing.x + legendSize.x, plotSize.y };
    ImVec2 canvasMin = ImGui::GetCursorScreenPos();

    ImGui::InvisibleButton("ProfileGraph", canvasSize);

    ImVec2 plotMin = canvasMin;
    ImVec2 plotMax{ plotMin.x + plotSize.x, plotMin.y + plotSize.y };
    ImVec2 legendMin{ plotMax.x + spacing.x, plotMin.y };
    ImU32 backgroundColor = IM_COL32(14, 29, 34, 255);
    ImU32 borderColor = IM_COL32(190, 205, 205, 255);
    ImU32 gridColor = IM_COL32(80, 105, 110, 90);

    drawList->AddRectFilled(plotMin, plotMax, backgroundColor);
    drawList->AddRect(plotMin, plotMax, borderColor);

    for (int32 i = 1; i < 4; ++i)
    {
        float y = plotMax.y - plotSize.y * (float)i / 4.0f;
        drawList->AddLine(ImVec2{ plotMin.x, y }, ImVec2{ plotMax.x, y }, gridColor);
    }

    float columnStep = plotSize.x / (float)count;
    float barWidth = (std::max)(1.0f, columnStep - 1.0f);
    float scale = plotSize.y / (maxValue - minValue);

    for (int32 i = 0; i < count; ++i)
    {
        int32 index = (int32)((profileReadIndex + i) & (profileCapacity - 1));
        const WorldProfile& profile = profiles[index];

        float x0 = plotMin.x + columnStep * (float)i;
        float x1 = (std::min)(x0 + barWidth, plotMax.x);
        float stack = 0.0f;

        for (int32 j = entryCount - 1; j >= 0; --j)
        {
            float value = GetProfileValue(profile, entries[j].value);
            if (value <= 0.0f)
            {
                continue;
            }

            float y0 = plotMax.y - ((stack + value) - minValue) * scale;
            float y1 = plotMax.y - (stack - minValue) * scale;
            y0 = Clamp(y0, plotMin.y, plotMax.y);
            y1 = Clamp(y1, plotMin.y, plotMax.y);
            drawList->AddRectFilled(ImVec2{ x0, y0 }, ImVec2{ x1, y1 }, ToImColor(entries[j].color));
            stack += value;
        }
    }

    int32 latestIndex = (int32)((profileReadIndex + count - 1) & (profileCapacity - 1));
    const WorldProfile& latestProfile = profiles[latestIndex];
    float textHeight = ImGui::GetTextLineHeight();
    float legendStep = (std::min)(textHeight + 2.0f, plotSize.y / (float)entryCount);
    float legendY = legendMin.y;

    for (int32 i = 0; i < entryCount; ++i)
    {
        float stack = 0.0f;
        for (int32 j = entryCount - 1; j > i; --j)
        {
            float stackValue = GetProfileValue(latestProfile, entries[j].value);
            if (showAverage)
            {
                stackValue = 0.0f;
                for (int32 k = 0; k < count; ++k)
                {
                    int32 index = (int32)((profileReadIndex + k) & (profileCapacity - 1));
                    stackValue += GetProfileValue(profiles[index], entries[j].value);
                }
                stackValue /= (float)count;
            }

            stack += stackValue;
        }

        float value = GetProfileValue(latestProfile, entries[i].value);
        if (showAverage)
        {
            value = 0.0f;
            for (int32 j = 0; j < count; ++j)
            {
                int32 index = (int32)((profileReadIndex + j) & (profileCapacity - 1));
                value += GetProfileValue(profiles[index], entries[i].value);
            }
            value /= (float)count;
        }

        ImU32 entryColor = ToImColor(entries[i].color);
        float lineY = legendY + textHeight * 0.5f;
        float stackY = plotMax.y - ((stack + value * 0.5f) - minValue) * scale;
        stackY = Clamp(stackY, plotMin.y, plotMax.y);

        drawList->AddLine(ImVec2{ plotMax.x, stackY }, ImVec2{ legendMin.x - 3.0f, lineY }, entryColor, 1.0f);
        drawList->AddRectFilled(
            ImVec2{ legendMin.x, legendY + 3.0f }, ImVec2{ legendMin.x + 10.0f, legendY + 13.0f }, entryColor
        );

        char text[128];
        std::snprintf(text, sizeof(text), "[%.3f ms] %s", value, entries[i].name);
        drawList->AddText(ImVec2{ legendMin.x + 14.0f, legendY }, entryColor, text);

        legendY += legendStep;
    }

    ImVec2 mousePosition = ImGui::GetIO().MousePos;
    bool plotHovered = ImGui::IsItemHovered() && mousePosition.x >= plotMin.x && mousePosition.x < plotMax.x &&
                       mousePosition.y >= plotMin.y && mousePosition.y < plotMax.y;

    if (showOverlay && plotHovered)
    {
        int32 hoverOffset = (int32)((mousePosition.x - plotMin.x) / columnStep);
        hoverOffset = Clamp(hoverOffset, 0, count - 1);
        int32 hoverIndex = (int32)((profileReadIndex + hoverOffset) & (profileCapacity - 1));
        const WorldProfile& hoverProfile = profiles[hoverIndex];

        float lineX = plotMin.x + columnStep * ((float)hoverOffset + 0.5f);
        drawList->AddLine(ImVec2{ lineX, plotMin.y }, ImVec2{ lineX, plotMax.y }, IM_COL32(255, 255, 255, 180), 1.0f);

        ImGui::BeginTooltip();
        ImGui::Text("-%d frames", count - hoverOffset - 1);
        ImGui::Separator();
        ImGui::Text("Total: %.3f ms", GetProfileTotal(hoverProfile, entries, entryCount));

        for (int32 i = 0; i < entryCount; ++i)
        {
            float value = GetProfileValue(hoverProfile, entries[i].value);
            ImGui::TextColored(
                ImVec4{
                    (float)((entries[i].color >> 16) & 0xff) / 255.0f,
                    (float)((entries[i].color >> 8) & 0xff) / 255.0f,
                    (float)(entries[i].color & 0xff) / 255.0f,
                    1.0f,
                },
                "%.3f ms %s", value, entries[i].name
            );
        }

        ImGui::EndTooltip();
    }

    ImGui::Text("Min %.3f ms, Max %.3f ms, Avg %.3f ms, Current %.3f ms", minTotal, maxTotal, sumTotal / (float)count, nowTotal);
}

} // namespace muli3
