#include "FieldWidget.h"
#include "../RobotController.h"
#include "../Strategies/AStarAttack.h"
#include "../SafeDrawing.h"
#include "../Input/InputState.h"
#include "CameraWidget.h"
#include "imgui.h"
#include <fstream>
#include <sstream>

FieldWidget* FieldWidget::_instance = nullptr;
bool FieldWidget::EditFieldBoundaries = false;
bool FieldWidget::_boundariesLoaded = false;
bool FieldWidget::_boundariesLoadedFromFile = false;

FieldWidget::FieldWidget() : ImageWidget("Field", false)
{
    _instance = this;
}

FieldWidget *FieldWidget::GetInstance()
{
    return _instance;
}

void FieldWidget::DrawFieldBoundaryEditor() {
    // Try to load saved boundaries on first access
    if (!_boundariesLoaded && AStarAttack::GetInstance()) {
        LoadFieldBoundariesIfExists();
        _boundariesLoaded = true;
    }
    
    // Only draw editing interface when editing is enabled
    if (EditFieldBoundaries) {
        _DrawFieldBoundaryPoints();
        _HandleFieldBoundaryInteraction();
    }
}

void FieldWidget::_DrawFieldBoundaryPoints() {
    AStarAttack* astar = AStarAttack::GetInstance();
    if (!astar) return;
    
    auto& boundaryPoints = astar->GetFieldBoundaryPoints();
    cv::Mat& drawingImage = RobotController::GetInstance().GetDrawingImage();
    
    const double HANDLE_RADIUS = 8.0;
    const double SELECTED_HANDLE_RADIUS = 12.0;
    
    for (int i = 0; i < boundaryPoints.size(); i++) {
        cv::Point2f point = boundaryPoints[i];
        
        // Choose color and size based on selection state
        cv::Scalar color;
        double radius;
        
        if (i == _selectedPointIndex) {
            color = cv::Scalar(0, 255, 255); // Yellow for selected
            radius = SELECTED_HANDLE_RADIUS;
        } else {
            color = cv::Scalar(255, 255, 0); // Cyan for normal
            radius = HANDLE_RADIUS;
        }
        
        // Draw the handle
        safe_circle(drawingImage, point, radius, color, 2);
        
        // Draw point index as text
        cv::putText(drawingImage, std::to_string(i), 
                   cv::Point(point.x + 10, point.y - 10), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
    }
}

void FieldWidget::_HandleFieldBoundaryInteraction() {
    if (!IsMouseOver()) return;
    
    AStarAttack* astar = AStarAttack::GetInstance();
    if (!astar) return;
    
    auto& boundaryPoints = astar->GetFieldBoundaryPoints();
    cv::Point2f mousePos = GetMousePos();
    
    const double HANDLE_RADIUS = 12.0; // Slightly larger for easier clicking
    
    // Handle mouse down - check for point selection
    if (InputState::GetInstance().IsMouseDown(0) && !_draggingPoint) {
        _selectedPointIndex = -1;
        
        // Find closest point within handle radius
        for (int i = 0; i < boundaryPoints.size(); i++) {
            double distance = cv::norm(mousePos - boundaryPoints[i]);
            if (distance <= HANDLE_RADIUS) {
                _selectedPointIndex = i;
                _draggingPoint = true;
                _lastMousePos = mousePos;
                break;
            }
        }
    }
    
    // Handle dragging
    if (_draggingPoint && InputState::GetInstance().IsMouseDown(0) && _selectedPointIndex >= 0) {
        cv::Point2f mouseDelta = mousePos - _lastMousePos;
        
        // Update the selected point position
        if (_selectedPointIndex < boundaryPoints.size()) {
            boundaryPoints[_selectedPointIndex] += mouseDelta;

            // force the first point to be the same as the last point
            if (_selectedPointIndex == 0) {
                boundaryPoints[boundaryPoints.size() - 1] = boundaryPoints[0];
            }
            if (_selectedPointIndex == boundaryPoints.size() - 1) {
                boundaryPoints[0] = boundaryPoints[boundaryPoints.size() - 1];
            }
            
            // Regenerate the boundary lines
            astar->RegenerateFieldBoundaryLines();
        }
        
        _lastMousePos = mousePos;
    }
    
    // Handle mouse release
    if (!InputState::GetInstance().IsMouseDown(0)) {
        _draggingPoint = false;
    }
}

void FieldWidget::_SaveFieldBoundaries(const std::string& filename) {
    AStarAttack* astar = AStarAttack::GetInstance();
    if (!astar) return;
    
    auto& points = astar->GetFieldBoundaryPoints();
    std::ofstream file(filename);
    
    if (file.is_open()) {
        file << "# Field Boundary Points\n";
        file << points.size() << "\n";
        
        for (const auto& point : points) {
            file << point.x << " " << point.y << "\n";
        }
        
        file.close();
    }
}

void FieldWidget::_LoadFieldBoundaries(const std::string& filename) {
    AStarAttack* astar = AStarAttack::GetInstance();
    if (!astar) return;
    
    std::ifstream file(filename);
    if (!file.is_open()) return;
    
    std::string line;
    // Skip comment line
    std::getline(file, line);
    
    // Read number of points
    int numPoints;
    if (!(file >> numPoints) || numPoints <= 0) return;
    
    std::vector<cv::Point2f> newPoints;
    newPoints.reserve(numPoints);
    
    // Read points
    for (int i = 0; i < numPoints; i++) {
        float x, y;
        if (file >> x >> y) {
            newPoints.emplace_back(x, y);
        }
    }
    
    if (newPoints.size() == numPoints) {
        astar->SetFieldBoundaryPoints(newPoints);
        _selectedPointIndex = -1;
        _draggingPoint = false;
    }
    
    file.close();
}

void FieldWidget::LoadFieldBoundariesIfExists() {
    // Try to load saved boundaries if they exist
    std::ifstream testFile("field_boundaries.txt");
    if (testFile.good()) {
        testFile.close();
        _LoadFieldBoundaries("field_boundaries.txt");
        _boundariesLoadedFromFile = true;
    } else {
        _boundariesLoadedFromFile = false;
    }
}

void FieldWidget::DrawGUI() {
    // Try to load saved boundaries on first GUI access
    if (!_boundariesLoaded && AStarAttack::GetInstance()) {
        LoadFieldBoundariesIfExists();
        _boundariesLoaded = true;
    }
    
    ImGui::Begin("Field Editor");
    
    ImGui::Checkbox("Edit Field Boundaries", &EditFieldBoundaries);
    
    if (EditFieldBoundaries) {
        ImGui::Text("Field Boundary Editor");
        ImGui::Text("Left-click and drag boundary points");
        
        ImGui::Separator();
        
        AStarAttack* astar = AStarAttack::GetInstance();
        if (astar) {
            // Control buttons row 1
            if (ImGui::Button("Reset to Default")) {
                astar->ResetFieldBoundariesToDefault();
                _selectedPointIndex = -1;
                _draggingPoint = false;
                _boundariesLoadedFromFile = false;
            }
            
            ImGui::SameLine();
            if (ImGui::Button("Clear Selection")) {
                _selectedPointIndex = -1;
                _draggingPoint = false;
            }
            
            // Control buttons row 2
            if (ImGui::Button("Save Boundaries")) {
                _SaveFieldBoundaries("field_boundaries.txt");
            }
            
            ImGui::SameLine();
            if (ImGui::Button("Load Boundaries")) {
                _LoadFieldBoundaries("field_boundaries.txt");
                _boundariesLoadedFromFile = true;
            }
            
            ImGui::Separator();
            
            // Status information
            auto& points = astar->GetFieldBoundaryPoints();
            ImGui::Text("Total Points: %d", (int)points.size());
            
            // Show if boundaries were loaded from file
            if (_boundariesLoadedFromFile) {
                ImGui::Text("Status: Boundaries loaded from file");
            } else {
                ImGui::Text("Status: Using default boundaries");
            }
            
            // Show selected point info
            if (_selectedPointIndex >= 0 && _selectedPointIndex < points.size()) {
                ImGui::Text("Selected Point %d: (%.1f, %.1f)", 
                           _selectedPointIndex, 
                           points[_selectedPointIndex].x, 
                           points[_selectedPointIndex].y);
            } else {
                ImGui::Text("No point selected");
            }
            
            // Mouse position info when over field
            if (IsMouseOver()) {
                cv::Point2f mousePos = GetMousePos();
                ImGui::Text("Mouse: (%.1f, %.1f)", mousePos.x, mousePos.y);
            }
        }
    }
    
    ImGui::End();
}
