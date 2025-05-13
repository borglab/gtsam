# Modernizing EasySLAM

**Overall Goal:** To update the `easySLAM` codebase to use modern GTSAM conventions (anno 2025), making it compatible with the latest GTSAM and C++17 standards.

**Key Tasks:**

1.  **Configuration:** Replace custom `Config` objects with `gtsam::Values`.
2.  **Factors:** Update custom factor classes to derive from modern `gtsam::NoiseModelFactorN` (or `gtsam::NoiseModelFactor`) and implement their interfaces correctly.
3.  **Linearization:** Ensure `linearize` methods return `std::shared_ptr<gtsam::GaussianFactor>`.
4.  **Homography:** Integrate `homography.cpp/h` into `gtsam/geometry` as a proper geometric type (manifold or group).
5.  **Data Types:** Use GTSAM's geometric types (e.g., `Point2`, `Pose3`) and C++ standard library types where appropriate.
6.  **Code Style & Safety:** Address `TODO`s, replace unsafe C-style constructs (like `sprintf` for string building) with safer C++ alternatives.

---

## Phase 1: Understanding the Old and New Worlds

**Rationale:** Before changing code, you need a solid grasp of what the old code does and how modern GTSAM handles similar concepts.

**Execution Steps:**

1.  **Study `easySLAM` Components:**
    *   Read through `ARRobotMarker.h/.cpp`: Understand how ARToolKit data is ingested and represented (corners, IDs, calibration, marker size). Pay attention to `estimate_pose()` and its use of homography.
    *   Read through `Marker.h/.cpp`: This defines the marker geometry and projection functions (`hGetP`, `hGetAll`, `DhGetP`). These are crucial for the measurement model.
    *   Read through `CameraMarkerFactor.h/.cpp` (and `Factor0`, `Factor1`): Understand what each factor connects (robot pose, marker pose), what is known, and what is estimated. Note the `error_vector` and `linearize` methods. The `dump()` methods are for a custom serialization.
    *   Read through `EasySLAMConfig.h/.cpp`: This is the old "Values" container. Note how it stores robot and marker poses and how it's initialized.
    *   Read through `EasySLAMGraph.h/.cpp`: This builds the factor graph using the custom factors and config.
2.  **Study Modern GTSAM Equivalents:**
    *   **`gtsam::Values`:** How it stores different types keyed by `gtsam::Key` (often created with `gtsam::Symbol`).
    *   **`gtsam::NonlinearFactor`:** The base class.
    *   **`gtsam::NoiseModelFactor`:** For factors with a noise model.
    *   **`gtsam::NoiseModelFactorN<VALUE_TYPES...>`:** The templated helper for N-way factors. Focus on:
        *   Constructor: Takes `SharedNoiseModel` and `Key`s.
        *   `evaluateError(const VALUE_TYPES&..., OptionalMatrixType H...) const`: The core method to implement, returning the error vector `h(x) - z` and optionally Jacobians.
    *   **`gtsam::GaussianFactor` / `gtsam::JacobianFactor`:** The result of `linearize()`. Understand how it's constructed from Jacobians, an error vector (b-term), and a noise model.
    *   **`gtsam::NonlinearFactorGraph`:** The modern container for factors.
    *   **Basic GTSAM Examples:** Look at how factors are defined and used in existing GTSAM examples.
3.  **Identify Key Mappings:**
    *   `FGConfig` (and `EasySLAMConfig`, `EasyVectorConfig`) maps to `gtsam::Values`.
    *   `CameraMarkerFactor::error_vector` maps to the error calculation part of `NoiseModelFactorN::evaluateError`.
    *   `CameraMarkerFactor::linearize` (returning `LinearFactor::shared_ptr`) maps to `NoiseModelFactor::linearize` (returning `std::shared_ptr<gtsam::GaussianFactor>`).
    *   String-based keys (`"x1"`, `"m1"`) map to `gtsam::Key` (e.g., `gtsam::Symbol('x', 1)`).

**Testing Strategy for this Phase:**
*   Not direct code testing, but self-assessment. Can you explain the role of each old component and its modern GTSAM counterpart?

---

## Phase 2: Modernizing the Factors (`CameraMarkerFactor`, `0`, `1`)

**Rationale:** Factors are the heart of the SLAM system. Updating them to the modern GTSAM interface is the most critical step.

**Execution Steps:**

1.  **Choose a Base Class:**
    *   `CameraMarkerFactor0`: Connects one robot pose (`Pose3`). Suitable for `gtsam::NoiseModelFactorN<gtsam::Pose3>`.
    *   `CameraMarkerFactor1`: Connects one marker pose (`Pose3`). Suitable for `gtsam::NoiseModelFactorN<gtsam::Pose3>`.
    *   `CameraMarkerFactor`: Connects a robot pose (`Pose3`) and a marker pose (`Pose3`). Suitable for `gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Pose3>`.
2.  **Update `CameraMarkerFactor0.h/.cpp` (Example):**
    *   **Includes:** Add necessary modern GTSAM headers.
    *   **Base Class:** Change `public gtsam::NonlinearFactor` to `public gtsam::NoiseModelFactorN<gtsam::Pose3>`.
    *   **Constructor:**
        *   Old: `CameraMarkerFactor0(const gtsam::Vector& z, double sigma, int rn, gtsam::Cal3_S2 K, int markerSize, const gtsam::Pose3& knownMarker)`
        *   New: `CameraMarkerFactor0(gtsam::Key robotKey, const gtsam::Vector& measured_z, const gtsam::SharedNoiseModel& noiseModel, const gtsam::Cal3_S2& K, double markerSize, const gtsam::Pose3& knownMarkerPose)`
            *   Store `measured_z_`, `K_`, `markerSize_`, `knownMarkerPose_` as member variables.
            *   Pass `noiseModel` and `robotKey` to the `NoiseModelFactorN` base constructor.
        *   Adapt the constructor that takes `ARRobotMarker`. It will now extract `robotKey` (e.g., `Symbol('x', marker.getRobotNumber())`), create a `noiseModel` (e.g., `noiseModel::Isotropic::Sigma(8, sigma)`), and then call the main constructor.
    *   **`evaluateError` Method:**
        *   Signature: `gtsam::Vector evaluateError(const gtsam::Pose3& robotPose, gtsam::OptionalMatrixType H_robot) const override;`
        *   Implementation:
            *   Use `gtsam::SimpleCamera camera(K_, robotPose);`
            *   Create `Marker markerObj(knownMarkerPose_, markerSize_);`
            *   Predicted measurement: `gtsam::Vector hx = hGetAll(camera, markerObj);`
            *   If `H_robot` is requested:
                *   Use `DhGetP` to get Jacobians for each of the 4 points with respect to the camera (robot) pose.
                *   Stack these Jacobians to form the `*H_robot` matrix (8x6).
            *   Return `hx - measured_z_`. (Note: GTSAM convention is typically `h(x) - z`. The old code has `z - h(x)`. Switch to `h(x) - z_` for consistency. The `b` term in `JacobianFactor` becomes `-(h(x)-z)` which is `z-h(x)`.)
    *   **`linearize` Method:** This method is now provided by `NoiseModelFactor` using `evaluateError`. You typically don't override it.
    *   **Remove Old Methods:** Delete `error_vector`, old `linearize`.
    *   **`equals` method:** Update to compare with the new structure.
    *   **`dump` method:** This custom serialization will likely need changes or be removed later. For now, adapt its string format if necessary.
3.  **Repeat for `CameraMarkerFactor1` and `CameraMarkerFactor`:** Follow a similar process, adjusting for the number of keys and variable types.
    *   For `CameraMarkerFactor`, `evaluateError` will be `(const gtsam::Pose3& robotPose, const gtsam::Pose3& markerPose, gtsam::OptionalMatrixType H_robot, gtsam::OptionalMatrixType H_marker) const`.
4.  **Key Management:** In the old code, keys were strings like `"x%d"`. In new code, use `gtsam::Symbol('x', robotNumber_)` and `gtsam::Symbol('m', markerNumber_)`.

**Testing Strategy:**

*   **Unit Tests for Each Factor:**
    *   Create simple test cases with known robot poses, marker poses, and calibration.
    *   Manually calculate the expected error vector.
    *   Use `factor.unwhitenedError(values)` (or `factor.evaluateError(...)` directly) and compare.
    *   **Crucially, use `GTSAM_EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, tolerance, step_size);`** to numerically verify your implemented Jacobians in `evaluateError`. This is found in `gtsam/nonlinear/tests/utilities.h` (or similar path depending on GTSAM version).
    *   Test `equals()` method.
*   **Noise Model:** Ensure the noise model is correctly passed and used.

---

## Phase 3: Modernizing Configuration (`EasySLAMConfig`, `EasyVectorConfig`)

**Rationale:** `gtsam::Values` is the standard, type-safe way to store variable estimates.

**Execution Steps:**

1.  **Eliminate `EasyVectorConfig`:** Its functionality is handled by `gtsam::Values` and `Pose3::retract`/`localCoordinates`.
2.  **Refactor `EasySLAMConfig` to `EasySLAMValues` (or just use `gtsam::Values` directly):**
    *   **Internal Storage:** Replace `poseMap markerPoses; poseMap robotPoses;` with a single `gtsam::Values values_;`.
    *   **Constructors:**
        *   The constructor from `FGConfig` becomes obsolete.
        *   The constructor `EasySLAMConfig(std::string& path, int num_of_frames)` will now call the `load` method which populates the internal `gtsam::Values`.
    *   **Methods:**
        *   `addRobotPose(int i, Pose3 rp)` becomes `values_.insert(gtsam::Symbol('x', i), rp);`. Similarly for `addMarkerPose`.
        *   `robotPoseExists(int i)` becomes `values_.exists(gtsam::Symbol('x', i));`.
        *   `robotPose(int i)` becomes `values_.at<gtsam::Pose3>(gtsam::Symbol('x', i));`.
        *   `getFGConfig()` is obsolete.
        *   `load(int refMarker, std::vector<ARRobotMarker*> measurements, Cal3_S2 K, double markerSize)`: This is the core initialization logic. It will now populate the `values_` object.
        *   `loadAFrame`: Similar update.
        *   `dump` and `load_dumped`: These are custom serialization. They need to be updated to read/write the `gtsam::Values`. Consider using GTSAM's built-in serialization.
        *   `transform_to(const gtsam::Pose3& transform)`: Iterate `values_`, get each `Pose3`, transform it, and insert into a new `gtsam::Values` object.
        *   `equals` and `assert_equal`: Update to work with `gtsam::Values`.
3.  **Update `ARRobotMarker.h`:**
    *   `std::vector<Vector> Points; // (TODO: should be Point2s)`: Change to `std::vector<gtsam::Point2> image_corners_;`. Update all constructors and accessors.
    *   `gtsam::Pose3 estimatedPose_; // Temporary pose for estimate, FD: fix this hack`: This initial estimate from homography is likely still useful for initializing `gtsam::Values`.

**Testing Strategy:**

*   **Unit Tests for `EasySLAMValues` (or its replacement logic):**
    *   Test adding, retrieving, checking existence of poses.
    *   Test `load` logic with mock `ARRobotMarker` data and verify the correct `gtsam::Values` are created.
    *   Test `transform_to`.
    *   Test serialization/deserialization.
*   **`ARRobotMarker` tests:**
    *   Test that `image_corners_` are correctly populated and accessed.

---

## Phase 4: Modernizing the Graph (`EasySLAMGraph`)

**Rationale:** `EasySLAMGraph` should simply be a `gtsam::NonlinearFactorGraph` populated with the modernized factors.

**Execution Steps:**

1.  **Simplify `EasySLAMGraph`:**
    *   It already derives from `gtsam::NonlinearFactorGraph`.
    *   The `load` methods (`load`, `loadAFrame`, `insertMarker`) will now:
        *   Take `ARRobotMarker*` data.
        *   Create instances of the modernized `CameraMarkerFactor`, `CameraMarkerFactor0`, `CameraMarkerFactor1`.
        *   Pass appropriate `Key`s, `SharedNoiseModel`, and other parameters.
        *   Call `this->push_back(factor_ptr);` to add to the graph.
    *   `dump` and `load_dumped`: These methods deal with custom factor serialization.
        *   The `dump` method calls `factors[i]->dump()`. If the factor's `dump()` was updated, this might still work.
        *   The `load_dumped` method parses this custom format. It will need to be updated to create the *modernized* factors.
        *   **Consider replacing these with GTSAM's native graph serialization** (`gtsam::writeGtsamGraph`, `gtsam::readGtsamGraph`) if feasible.
2.  **Main Application Logic (Implied):** The code that *uses* `EasySLAMGraph` and `EasySLAMConfig` will need to be updated to pass `gtsam::Values` to the optimizer.

**Testing Strategy:**

*   **Graph Construction Tests:**
    *   Provide mock `ARRobotMarker` data to `EasySLAMGraph::load` methods.
    *   Verify that the graph contains the correct number and types of factors, connected to the correct keys.
    *   Check that noise models and other factor parameters are correctly set.
*   **Serialization Tests:** If custom `dump/load_dumped` is kept and adapted, test that a graph can be saved and loaded, and is identical to the original.

---

## Phase 5: Homography Integration (`homography.h/.cpp`)

**Rationale:** Homography is a general geometric concept and should reside in `gtsam/geometry`. It's a 3x3 matrix (PGL(3,R)).

**Execution Steps:**

1.  **Move Files:** Move `homography.h` and `homography.cpp` to `gtsam/geometry/`.
2.  **Create `gtsam::Homography2D` Class:**
    *   Header: `gtsam/geometry/Homography2D.h`.
    *   Store the 3x3 matrix: `gtsam::Matrix3 H_;`.
    *   **Constructors:** Default, from `Matrix3`, copy.
    *   **Manifold/Group Properties:**
        *   `dimension()`: 8 (since H is defined up to scale).
        *   `print(const std::string& s = "") const`.
        *   `equals(const Homography2D& other, double tol = 1e-9) const`.
        *   **Lie Group Operations (Optional but good):** `Identity()`, `inverse()`, `compose()`, `retract()`, `localCoordinates()`. This requires careful handling of the scale ambiguity.
    *   **Normalization:** Add a private method `normalize()` (e.g., divide by `H_(2,2)` or Frobenius norm).
    *   **Utility Functions (Static or Member):**
        *   `static Homography2D Estimate(const std::vector<Point2>& p1, const std::vector<Point2>& p2);` (encapsulating `getL` and `getH`).
        *   `Pose3 decomposeHomography(const Cal3_S2& K) const;` (replaces `getTransformationFromMarkerToCamera`).
    *   **`fixToRotation_zhang`:** Replace with `gtsam::Rot3::ClosestTo(const Matrix3& M)`.
3.  **Update `ARRobotMarker::estimate_H()` and `estimate_pose()`:**
    *   Use `gtsam::Homography2D::Estimate(...)` and `homography.decomposeHomography(K_)`.
4.  **CMake:** Add the new files to `gtsam/geometry/CMakeLists.txt`.

**Testing Strategy:**

*   **Unit Tests for `Homography2D`:**
    *   Test constructor, `equals`, `print`.
    *   Test Lie group operations if implemented.
    *   Test `Estimate` with known 2D point correspondences.
    *   Test `decomposeHomography` with a known homography.
    *   Verify `Rot3::ClosestTo()` against `fixToRotation_zhang`.

---

## Phase 6: Utilities and Final Cleanup

**Rationale:** Remove obsolete code, replace C-style utilities with GTSAM/C++ equivalents, and address TODOs.

**Execution Steps:**

1.  **`utility.h/.cpp` Review:**
    *   `det3(Matrix M)`: Use `M.determinant()`.
    *   `l2norm(Vector V)`: Use `V.norm()`.
    *   `transformationVectorToMatrix`, `transformationMatrixToVector`: Evaluate if still needed. `gtsam::Pose3` has `Expmap`/`Logmap`.
    *   `getRotationFromGivens`: Use `gtsam::Rot3::RzRyRx()`.
    *   `Drot3`: Analyze its purpose. `Rot3` provides Jacobians for its operations. `skewSymmetric()` is standard.
    *   `rotationTranslation2Transformation`: `gtsam::Pose3(R, t)` constructor.
    *   `CantOpenFile`: Use `std::runtime_error` or `std::ifstream::failure`.
2.  **Replace `sprintf` and `strncat` in `dump()` methods:** Use `std::ostringstream` for safer string construction.
    ```cpp
    std::ostringstream oss;
    oss << "1 " << robotNumber_ << " " << markerNumber_ << " " << noiseModel_->sigma(0) << " " << measured_z_.size();
    for (size_t k = 0; k < measured_z_.size(); ++k) {
        oss << " " << measured_z_(k);
    }
    // ...
    return oss.str();
    ```
3.  **Address TODOs:** Systematically review and resolve comments like `// TODO: ...` or `// FD says: ...`.
4.  **Review Includes:** Ensure all necessary headers are included and obsolete ones are removed.
5.  **Code Formatting:** Run a code formatter (like clang-format with GTSAM's style).

**Testing Strategy:**

*   Relies heavily on tests from previous phases.
*   If `dump()` methods were changed, test their output or the `load_dumped()` functionality.

---

## Phase 7: Integration and Full System Test

**Rationale:** Ensure all modernized components work together in the context of the original `easySLAM` application.

**Execution Steps:**

1.  **Create a Main Executable:**
    *   Loads ARToolKit data using `ARRobotMarker`'s loading functions.
    *   Initializes `gtsam::Values`.
    *   Builds the `gtsam::NonlinearFactorGraph`.
    *   Adds priors if necessary.
    *   Creates a GTSAM optimizer (e.g., `gtsam::LevenbergMarquardtOptimizer`).
    *   Runs `optimizer.optimize()`.
    *   Retrieves and processes results from the optimized `gtsam::Values`.
2.  **Compare with Original (if possible):** If the old `easySLAM` can be run, try to compare its output on a dataset with the modernized version's output.

**Testing Strategy:**

*   **End-to-End Test:** Run the full pipeline on one or more datasets.
    *   Does it run without crashing?
    *   Are the results (estimated poses) reasonable?
    *   Is the error decreasing during optimization?
*   **Ground Truth Comparison:** If any datasets have ground truth, compare the estimated trajectory/map against it.

---

## General Advice for you:

*   **Git is Your Friend:** Commit frequently with clear messages. Use branches for significant features or refactoring.
*   **Start Small:** Begin with the simplest factor (`CameraMarkerFactor0`) to get comfortable with the new interfaces.
*   **Compile Often:** Catch errors early.
*   **Read GTSAM Documentation:** The GTSAM docs and Doxygen are valuable resources.
*   **Look at GTSAM Tests:** The tests in GTSAM (e.g., `gtsam/gtsam/nonlinear/tests`) are excellent examples.
*   **Ask Questions:** When stuck, ask for help. Explain what you've tried and where the problem lies.
*   **Patience:** Modernizing legacy code can be challenging but is a very valuable learning experience.