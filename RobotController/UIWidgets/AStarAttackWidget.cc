#include "AStarAttackWidget.h"

#include <algorithm>
#include <limits>
#include <string>
#include <imgui.h>

#include "../Strategies/AStarAttack.h"
#include "../Strategies/FollowPoint.h"

namespace {

void CenterText(const char* text) {
  if (!text) return;
  float cellWidth = ImGui::GetColumnWidth();
  float textWidth = ImGui::CalcTextSize(text).x;
  float offsetX = (cellWidth - textWidth) * 0.5f;
  if (offsetX > 0.0f) {
    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + offsetX);
  }
  ImGui::TextUnformatted(text);
}

}  // namespace

void AStarAttackWidget::Draw() {
  if (!ImGui::Begin("A* Attack Scores")) {
    ImGui::End();
    return;
  }

  AStarAttack* attack = AStarAttack::GetInstance();
  if (!attack) {
    ImGui::TextUnformatted("AStarAttack not initialized.");
    ImGui::End();
    return;
  }

  const std::vector<FollowPoint>& follows = attack->GetFollowPoints();
  if (follows.empty()) {
    ImGui::TextUnformatted("No follow points computed yet.");
    ImGui::End();
    return;
  }

  const int numCols = static_cast<int>(follows.size());

  // Use the names from the first follow point to label score rows.
  const auto& scoreNames = follows.front().directionScoreNames;

  // Find the column (follow point) with the lowest total score
  int bestCol = -1;
  float bestScore = std::numeric_limits<float>::infinity();
  for (int col = 0; col < numCols; ++col) {
    const auto& scores = follows[col].directionScores;
    if (!scores.empty()) {
      float total = scores.back();
      if (total < bestScore) {
        bestScore = total;
        bestCol = col;
      }
    }
  }

  // Compute a pulsing alpha for the best column highlight at 0.7 Hz
  const float pulseHz = 0.7f;
  const float t = static_cast<float>(ImGui::GetTime());
  const float phase = 0.5f + 0.5f * std::sin(2.0f * 3.1415926535f * pulseHz * t);  // 0..1
  const int headerAlpha = static_cast<int>(40.0f + 60.0f * phase);  // 40..100
  const int cellAlpha   = static_cast<int>(30.0f + 50.0f * phase);  // 30..80

  // Find the maximum number of score entries across all follow points
  int maxScores = 0;
  for (const auto& fp : follows) {
    maxScores = std::max(maxScores,
                         static_cast<int>(fp.directionScores.size()));
  }
  if (maxScores == 0) {
    ImGui::TextUnformatted("Follow points have no directionScores yet.");
    ImGui::End();
    return;
  }

  ImGuiTableFlags flags = ImGuiTableFlags_RowBg |
                          ImGuiTableFlags_Borders |
                          ImGuiTableFlags_SizingStretchProp |
                          ImGuiTableFlags_ScrollX |
                          ImGuiTableFlags_ScrollY;

  // First column is the score term label, remaining columns are followPoints.
  if (ImGui::BeginTable("##astar_scores", 1 + numCols, flags)) {
    ImGui::TableSetupColumn("Term", ImGuiTableColumnFlags_WidthFixed, 90.0f);

    for (int col = 0; col < numCols; ++col) {
      ImGui::TableSetupColumn(nullptr, ImGuiTableColumnFlags_WidthStretch, 1.0f);
    }

    // Header row: show one column per followPoint.
    ImGui::TableHeadersRow();
    ImGui::TableNextRow();

    // Term label column header
    ImGui::TableSetColumnIndex(0);
    ImGui::TextUnformatted("Component");

    // FollowPoint column headers: summarize CW/CCW, For/Back, turnAway/natural.
    for (int col = 0; col < numCols; ++col) {
      ImGui::TableSetColumnIndex(1 + col);

      const FollowPoint& fp = follows[col];
      std::string header;
      header += fp.CW ? "CW" : "CCW";
      header += " ";
      header += fp.forward ? "For" : "Back";
      header += " ";
      header += fp.turnAway ? "Away" : "Natural";

      if (col == bestCol) {
        ImGui::TableSetBgColor(ImGuiTableBgTarget_CellBg,
                               IM_COL32(100, 200, 100, headerAlpha));
      }

      ImGui::TextUnformatted(header.c_str());
    }

    // Body rows: one row per score component index.
    for (int row = 0; row < maxScores; ++row) {
      ImGui::TableNextRow();

      // First column: component label (from directionScoreNames where available)
      ImGui::TableSetColumnIndex(0);
      if (row < static_cast<int>(scoreNames.size())) {
        ImGui::TextUnformatted(scoreNames[row].c_str());
      } else if (row == maxScores - 1) {
        ImGui::TextUnformatted("Total");
      } else {
        ImGui::Text("Term %d", row);
      }

      // One cell per followPoint
      for (int col = 0; col < numCols; ++col) {
        ImGui::TableSetColumnIndex(1 + col);

        const auto& scores = follows[col].directionScores;
        if (col == bestCol) {
          ImGui::TableSetBgColor(ImGuiTableBgTarget_CellBg,
                                 IM_COL32(100, 200, 100, cellAlpha));
        }

        if (row < static_cast<int>(scores.size())) {
          char buf[32];
          snprintf(buf, sizeof(buf), "%.1f", scores[row]);
          CenterText(buf);
        } else {
          CenterText("-");
        }
      }
    }

    ImGui::EndTable();
  }

  ImGui::End();
}

