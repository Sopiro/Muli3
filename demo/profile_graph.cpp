#include "profile_graph.h"

namespace muli3
{

static ImU32 ToImColor(uint32 c)
{
    return IM_COL32((c >> 16) & 0xff, (c >> 8) & 0xff, c & 0xff, 255);
}

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
)
{
    float maxTotal = 0.0f;
    float minTotal = 0.0f;
    float sumTotal = 0.0f;
    float nowTotal = 0.0f;

    for (int32 i = 0; i < count; ++i)
    {
        float total = 0.0f;
        for (int32 j = 0; j < entryCount; ++j)
        {
            total += entries[j].values[i];
        }

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
    float maxValue = count > 0 ? (std::max)(maxTotal, 0.1f) : 0.1f;
    maxValue = (std::max)(maxValue, 0.001f);

    if (strlen(label) > 0)
    {
        ImGui::Text("%s", label);
    }

    ImDrawList* drawList = ImGui::GetWindowDrawList();
    ImVec2 avail = ImGui::GetContentRegionAvail();
    ImVec2 plotSize{ graphSize.x, graphSize.y };
    ImVec2 spacing{ 14.0f, 0.0f };
    ImVec2 legendSize{ 240.0f, 0.0f };

    if (plotSize.x < 0.0f)
    {
        plotSize.x = (std::max)(180.0f, avail.x - legendSize.x - spacing.x);
    }

    if (plotSize.y < 0.0f)
    {
        plotSize.y = Clamp(plotSize.x * 0.38f, 120.0f, 220.0f);
    }

    legendSize.y = plotSize.y;
    ImVec2 canvasSize{ plotSize.x + spacing.x + legendSize.x, plotSize.y };
    ImVec2 canvasMin = ImGui::GetCursorScreenPos();

    ImGui::InvisibleButton("ProfileGraph", canvasSize);

    ImVec2 plotMin = canvasMin;
    ImVec2 plotMax{ plotMin.x + plotSize.x, plotMin.y + plotSize.y };
    ImVec2 legendMin{ plotMax.x + spacing.x, plotMin.y };
    ImU32 backgroundColor = IM_COL32(14, 29, 34, 255);
    ImU32 borderColor = IM_COL32(190, 205, 205, 255);
    ImU32 gridColor = IM_COL32(80, 105, 110, 90);
    float axisLabelWidth = showAxisLabels ? 32.0f : 0.0f;
    ImVec2 graphMin{ plotMin.x + axisLabelWidth, plotMin.y };
    ImVec2 graphMax = plotMax;

    drawList->AddRectFilled(graphMin, graphMax, backgroundColor);
    drawList->AddRect(graphMin, graphMax, borderColor);

    for (int32 i = 1; i < 4; ++i)
    {
        float y = graphMax.y - plotSize.y * (float)i / 4.0f;
        drawList->AddLine(ImVec2{ graphMin.x, y }, ImVec2{ graphMax.x, y }, gridColor);
    }

    for (int32 i = 0; i <= 4 && showAxisLabels; ++i)
    {
        float t = (float)i / 4.0f;
        float y = graphMax.y - plotSize.y * t;
        float value = maxValue * t;
        char text[32];
        std::snprintf(text, sizeof(text), "%.2f", value);
        ImVec2 size = ImGui::CalcTextSize(text);
        drawList->AddText(ImVec2{ graphMin.x - 4.0f - size.x, y - size.y * 0.5f }, borderColor, text);
    }

    float graphWidth = graphMax.x - graphMin.x;
    float columnStep = graphWidth / (float)profileCapacity;
    float barWidth = (std::max)(1.0f, columnStep - 1.0f);
    float scale = plotSize.y / (maxValue - minValue);
    float innerLeft = graphMin.x + 1.0f;
    float innerRight = graphMax.x - 1.0f;
    float innerTop = graphMin.y + 1.0f;
    float innerBottom = graphMax.y - 1.0f;
    int32 startSlot = profileCapacity - count;

    for (int32 i = 0; i < count; ++i)
    {
        float x0 = innerLeft + columnStep * (float)(startSlot + i);
        float x1 = (std::min)(x0 + barWidth, innerRight);
        float stack = 0.0f;

        for (int32 j = entryCount - 1; j >= 0; --j)
        {
            float value = entries[j].values[i];
            if (value <= 0.0f)
            {
                continue;
            }

            float y0 = plotMax.y - ((stack + value) - minValue) * scale;
            float y1 = plotMax.y - (stack - minValue) * scale;
            y0 = Clamp(y0, innerTop, innerBottom);
            y1 = Clamp(y1, innerTop, innerBottom);
            drawList->AddRectFilled(ImVec2{ x0, y0 }, ImVec2{ x1, y1 }, ToImColor(entries[j].color));
            stack += value;
        }
    }

    float textHeight = ImGui::GetTextLineHeight();
    float legendStep = (std::min)(textHeight + 2.0f, plotSize.y / (float)entryCount);
    float legendY = legendMin.y;

    for (int32 i = 0; i < entryCount; ++i)
    {
        float stack = 0.0f;
        for (int32 j = entryCount - 1; j > i; --j)
        {
            float stackValue = count > 0 ? entries[j].values[count - 1] : 0.0f;
            if (showAverage)
            {
                stackValue = 0.0f;
                for (int32 k = 0; k < count; ++k)
                {
                    stackValue += entries[j].values[k];
                }
                if (count > 0)
                {
                    stackValue /= (float)count;
                }
            }

            stack += stackValue;
        }

        float value = count > 0 ? entries[i].values[count - 1] : 0.0f;
        if (showAverage)
        {
            value = 0.0f;
            for (int32 j = 0; j < count; ++j)
            {
                value += entries[i].values[j];
            }
            if (count > 0)
            {
                value /= (float)count;
            }
        }

        ImU32 entryColor = ToImColor(entries[i].color);
        float lineY = legendY + textHeight * 0.5f;
        float ratio = (stack + value * 0.5f - minValue) / (maxValue - minValue);
        float stackY = graphMax.y - ratio * plotSize.y;
        stackY = Clamp(stackY, graphMin.y, graphMax.y);

        drawList->AddLine(ImVec2{ graphMax.x, stackY }, ImVec2{ legendMin.x - 3.0f, lineY }, entryColor, 1.0f);
        drawList->AddRectFilled(
            ImVec2{ legendMin.x, legendY + 3.0f }, ImVec2{ legendMin.x + 10.0f, legendY + 13.0f }, entryColor
        );

        char text[128];
        std::snprintf(text, sizeof(text), "[%.3f ms] %s", value, entries[i].name);
        drawList->AddText(ImVec2{ legendMin.x + 14.0f, legendY }, entryColor, text);

        legendY += legendStep;
    }

    ImVec2 mousePosition = ImGui::GetIO().MousePos;
    bool plotHovered = count > 0 && ImGui::IsItemHovered() && mousePosition.x >= innerLeft + columnStep * startSlot &&
                       mousePosition.x < innerLeft + columnStep * profileCapacity && mousePosition.y >= graphMin.y &&
                       mousePosition.y < graphMax.y;

    if (showOverlay && plotHovered)
    {
        int32 hoverSlot = (int32)((mousePosition.x - innerLeft) / columnStep);
        int32 hoverOffset = hoverSlot - startSlot;
        hoverOffset = Clamp(hoverOffset, 0, count - 1);

        float lineX = innerLeft + columnStep * ((float)(startSlot + hoverOffset) + 0.5f);
        drawList->AddLine(ImVec2{ lineX, graphMin.y }, ImVec2{ lineX, graphMax.y }, IM_COL32(255, 255, 255, 180), 1.0f);

        ImGui::BeginTooltip();
        ImGui::Text("-%d frames", count - hoverOffset - 1);
        ImGui::Separator();
        float total = 0.0f;
        for (int32 i = 0; i < entryCount; ++i)
        {
            total += entries[i].values[hoverOffset];
        }
        ImGui::Text("Total: %.3f ms", total);

        for (int32 i = 0; i < entryCount; ++i)
        {
            float value = entries[i].values[hoverOffset];
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

    float avgTotal = count > 0 ? sumTotal / (float)count : 0.0f;
    if (showAxisLabels)
    {
        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + axisLabelWidth);
    }
    ImGui::Text("Min %.3f ms, Max %.3f ms, Avg %.3f ms, Current %.3f ms", minTotal, maxTotal, avgTotal, nowTotal);
}

} // namespace muli3
