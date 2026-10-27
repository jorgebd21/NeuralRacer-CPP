#include "Car.h"
#include "Telemetry.h"
#include "FlatSpatialGrid.h"
#include <cassert>
#include <iostream>
#include <fstream>
#include <sstream>

void TestThermalTelemetryCapture() {
    Car car(100.0f, 100.0f, 0.0f);
    FlatSpatialGrid grid(1000.0f, 1000.0f, 50.0f);
    grid.SetCellSurface(2, 2, SurfaceType::Asphalt);

    std::vector<Vector2> checkpoints;

    assert(car.currentLapSamples.empty());

    // Run 10 ticks
    for (int t = 0; t < 10; ++t) {
        car.UpdatePhysics(1.0f, 0.2f, grid, t, checkpoints);
    }

    assert(car.currentLapSamples.size() == 10);
    const auto& sample = car.currentLapSamples[9];
    assert(sample.tireSurfaceTemp > 0.0f);
    assert(sample.tireBulkTemp > 0.0f);
    assert(sample.tireThermalGrip > 0.0f && sample.tireThermalGrip <= 1.0f);
    assert(sample.surfaceFriction > 0.0f);

    std::cout << "[PASS] TestThermalTelemetryCapture passed (Tire Surf: " 
              << sample.tireSurfaceTemp << " C, Bulk: " << sample.tireBulkTemp 
              << " C, Friction: " << sample.surfaceFriction << ")." << std::endl;
}

void TestTelemetryCsvExport() {
    Car car(150.0f, 150.0f, 0.0f);
    FlatSpatialGrid grid(1000.0f, 1000.0f, 50.0f);
    std::vector<Vector2> checkpoints;

    for (int t = 0; t < 5; ++t) {
        car.UpdatePhysics(0.8f, 0.1f, grid, t, checkpoints);
    }

    Telemetry::Init();
    std::vector<TelemetrySample> samples(car.currentLapSamples.begin(), car.currentLapSamples.end());
    Telemetry::RecordFastestLap(samples);

    std::string testCsvPath = "build/test_fastest_lap.csv";
    Telemetry::ExportFastestLapAsync(testCsvPath);
    Telemetry::Flush("build/test_telemetry_dummy.csv");

    // Read back and verify CSV
    std::ifstream file(testCsvPath);
    assert(file.is_open());
    std::string header;
    std::getline(file, header);
    assert(header.find("tire_surf_temp") != std::string::npos);
    assert(header.find("tire_bulk_temp") != std::string::npos);
    assert(header.find("tire_thermal_grip") != std::string::npos);
    assert(header.find("surface_friction") != std::string::npos);

    std::string row;
    int rowCount = 0;
    while (std::getline(file, row)) {
        if (!row.empty()) rowCount++;
    }
    assert(rowCount == 5);
    file.close();

    std::cout << "[PASS] TestTelemetryCsvExport passed with 5 valid 60Hz telemetry rows." << std::endl;
}

int main() {
    std::cout << "Running Thermal Telemetry Tests..." << std::endl;
    TestThermalTelemetryCapture();
    TestTelemetryCsvExport();
    std::cout << "All Thermal Telemetry tests completed successfully!" << std::endl;
    return 0;
}
