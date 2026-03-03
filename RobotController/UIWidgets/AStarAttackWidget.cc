#include "AStarAttackWidget.h"

#include <algorithm>
#include <imgui.h>

#include "../Strategies/AStarAttack.h"
#include "../Strategies/FollowPoint.h"

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

    // FollowPoint column headers
    for (int col = 0; col < numCols; ++col) {
      ImGui::TableSetColumnIndex(1 + col);

      const FollowPoint& fp = follows[col];
      const char* fwd = fp.forward ? "Fwd" : "Rev";
      const char* cw = fp.CW ? "CW" : "CCW";

      ImGui::Text("FP %d (%s, %s)", col, fwd, cw);
    }

    // Body rows: one row per score component index.
    for (int row = 0; row < maxScores; ++row) {
      ImGui::TableNextRow();

      // First column: component label
      ImGui::TableSetColumnIndex(0);
      if (row == maxScores - 1) {
        ImGui::TextUnformatted("Total");
      } else {
        ImGui::Text("Term %d", row);
      }

      // One cell per followPoint
      for (int col = 0; col < numCols; ++col) {
        ImGui::TableSetColumnIndex(1 + col);

        const auto& scores = follows[col].directionScores;
        if (row < static_cast<int>(scores.size())) {
          ImGui::Text("%.3f", scores[row]);
        } else {
          ImGui::TextUnformatted("-");
        }
      }
    }

    ImGui::EndTable();
  }

  ImGui::End();
}

