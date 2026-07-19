# PyRPOD Test Dashboard

This dashboard provides an overview of all tests in the PyRPOD project, categorized by their respective modules. Each test is listed with its type, description, and current status.

---

## **MDAO Tests**
| Test Name                       | Type           | Description                                   | Status |
|---------------------------------|----------------|-----------------------------------------------|--------|
| `mdao_unit_test_01.py`          | Unit           | empty.                                                              | ❌     |
| `mdao_unit_test_02.py`          | Unit           | Generate RCS configs by sweeping cant angle.                        | ⏳     |
| `mdao_integration_test_01.py`   | Integration    | empty.                                                              | ❌     |
| `mdao_verification_test_01.py`  | Verification   | Minimizes heat flux by varying axial position of RCS pack.          | ⏳     |
| `mdao_verification_test_02.py`  | Verification   | Minimizes heat flux by varying cant angle of RCS pack.              | ⏳     |
| `mdao_verification_test_03.py`  | Verification   | Evaluates RCS performance by varying cant angle and axial position. | ⏳     |
| `mdao_verification_test_04.py`  | Verification   | Evaluates RCS performance in axial-overshoot study.                 | ⏳     |
| `mdao_verification_test_05.py`  | Verification   | ????? no idea.                                                      | ❌     |
| `mdao_verification_test_06.py`  | Verification   | axial over shoot?                                                   | ❌     |

---

## **Mission Tests**
| Test Name                       | Type           | Description                                   | Status |
|---------------------------------|----------------|-----------------------------------------------|--------|
| `mission_integration_test_01.py`| Integration    | Assesrts 6DOF performance of individual thrusters.     | ⏳     |
| `mission_integration_test_02.py`| Integration    | Assesrts 6DOF performance of thruster groups.          | ⏳     |
| `mission_integration_test_03.py`| Integration    | Assersts Δv requirements for an RCS system (1D).       | ⏳     |
| `mission_integration_test_04.py`| Integration    | Assersts Δv requirements for an RCS system (2D).       | ⏳     |
| `mission_integration_test_05.py`| Integration    | Assersts thrust requirements for an RCS system.        | ⏳     |
| `mission_integration_test_06.py`| Integration    | Assersts mass requirements for an RCS system.          | ⏳     |
| `mission_integration_test_07.py`| Integration    | Contours RCS performance (thrust vs ISP).              | ⏳     |
| `mission_integration_test_08.py`| Integration    | Contours propellant usage for all Δv in a flight plan. | ⏳     |
| `mission_unit_test_01.py`       | Unit           | empty.                                                 | ❌     |
| `mission_verification_test_01.py`| Verification  | empty.                                                 | ❌     |

---

## **Plume Tests**
| Test Name                       | Type           | Description                                   | Status |
|---------------------------------|----------------|-----------------------------------------------|--------|
| `plume_integration_test_01.py`  | Integration    | Tests plume modeling in integrated systems.  | ❌     |
| `plume_unit_test_01.py`         | Unit           | Verifies individual plume calculation methods.| ❌     |
| `plume_verification_test_01.py` | Verification   | Validates plume outputs against benchmarks.  | ❌     |
| `plume_verification_test_28.py` | Verification   | Cai 2016 Fig. 17: diffuse-plate Cp contours (manual-run figure). | ✅     |
| `plume_verification_test_29.py` | Verification   | Cai 2016 Fig. 18: specular-plate Cp contours (manual-run figure). | ✅     |
| `plume_verification_test_30.py` | Verification   | Cai 2016 Fig. 19: diffuse-plate Cf1 contours (manual-run figure). | ✅     |
| `plume_verification_test_31.py` | Verification   | Cai 2016 Fig. 20: diffuse-plate Cf2 contours (manual-run figure). | ✅     |
| `plume_verification_test_32.py` | Verification   | Cai 2016 Fig. 21: diffuse-plate Cq contours (manual-run figure). | ✅     |
| `plume_verification_test_33.py` | Verification   | Cai 2016 Fig. 5: 2D diffuse-plate flowfield T contours (manual-run figure). | ✅     |
| `plume_verification_test_34.py` | Verification   | Cai 2016 Fig. 6: 2D specular-plate flowfield T contours (manual-run figure). | ✅     |
| `plume_verification_test_35.py` | Verification   | Cai 2016 Fig. 7: 2D diffuse-plate Cp profiles (manual-run figure). | ✅     |
| `plume_verification_test_36.py` | Verification   | Cai 2016 Fig. 8: 2D specular-plate Cp profiles (manual-run figure). | ✅     |
| `plume_verification_test_37.py` | Verification   | Cai 2016 Fig. 9: 2D diffuse-plate Cf profiles (manual-run figure). | ✅     |
| `plume_verification_test_38.py` | Verification   | Cai 2016 Fig. 10: 2D diffuse-plate Cq profiles (manual-run figure). | ✅     |
| `plume_verification_test_39.py` | Verification   | Cai 2016 Fig. 15: 3D diffuse-plate flowfield p contours (manual-run figure). | ✅     |
| `plume_verification_test_40.py` | Verification   | Cai 2016 Fig. 16: 3D specular-plate flowfield p contours (manual-run figure). | ✅     |
| `plume_impingement_error_summary.py` | Verification | Cai 2016 reference vs PyRPOD-chain max/mean error table (manual-run generator). | ✅     |

---

## **RPOD Tests**
| Test Name                       | Type           | Description                                   | Status |
|---------------------------------|----------------|-----------------------------------------------|--------|
| `rpod_integration_test_01.py`   | Integration    | Asserts plume strikes for RPOD "base case".          | ✅     |
| `rpod_integration_test_02.py`   | Integration    | Asserts plume strikes for notional 1D approach.      | ✅     |
| `rpod_integration_test_03.py`   | Integration    | Asserts plume strikes using KOZ geometry.            | ✅     |
| `rpod_integration_test_04.py`   | Integration    | Asserts plume strikes using hollow cube geometry.    | ✅     |
| `rpod_integration_test_07.py`   | Integration    | Cai 2016 inclined-plate case: pipeline loads vs exact reference. | ✅     |
| `rpod_unit_test_01.py`          | Unit           | Verifies STL to VTK data conversion.                 | ✅     |
| `rpod_unit_test_02.py`          | Unit           | Verifiy behavior of JFH reader.                      | ✅     |
| `rpod_unit_test_03.py`          | Unit           | Produces JFH data according to produced equation.    | ⏳     |
| `rpod_verification_test_03.py`  | Verification   | Tests strike counting from multiple thrusters.       | ⏳     |
| `rpod_verification_test_04.py`  | Verification   | Asserts plume strikes after TCD decoupling.          | ⏳     |

---

## **Legacy/Old Tests**
| Test Name                       | Type           | Description                                   | Status |
|---------------------------------|----------------|-----------------------------------------------|--------|
| `test_case_15.py`               | Miscellaneous  | Old test for deprecated functionality.       | ⏳     |
| `test_case_17.py`               | Miscellaneous  | Tests legacy feature interactions.           | ⏳     |
| `test_case_19.py`               | Miscellaneous  | Validates compatibility of old methods.      | ⏳     |
| `test_case_sweep_cants.py`      | Sweep Test     | Evaluates various canting angles.            | ⏳     |
| `test_case_sweep_coords.py`     | Sweep Test     | Tests coordinate system transformations.     | ⏳     |

---

## **Status Legend**
- ✅ = Passed
- ❌ = Not Started
- ⏳ = In Progress
- ⚙️ = Under Review
- 🛠️ = Requires Fixes

This dashboard serves as a quick reference for test organization and tracking within PyRPOD. Update the statuses regularly to ensure it reflects the latest testing outcomes.
