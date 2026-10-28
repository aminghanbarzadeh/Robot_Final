1. **Set up SDK Directory Structure:**
   - Create `SDK` root folder.
   - Create `SDK/include` and `SDK/lib` for standard library linkage.
   - Create `SDK/bin` for any provided utilities/executables.
   - Create `SDK/CustomController` to hold the template code that users will write.

2. **Copy Core Headers to `SDK/include`:**
   - Copy headers (`*.h`, `*.hpp`, `*.hh`) from `common/include/`, `robot/include/`, `third-party/` and `lcm-types/cpp/` to `SDK/include/` so they can compile custom controllers. Note: we might just preserve the source structure for `#include` directives to work without modification, or copy the specific directories in full. A safe choice is:
     `cp -r common/include/* SDK/include/`
     `cp -r robot/include/* SDK/include/`
     `cp -r lcm-types/cpp/* SDK/include/`
     `cp -r third-party SDK/include/` (to keep vectornav, osqp, inih headers intact)
   - *Alternative simpler strategy:* Since the user wants an SDK based on the codebase, we'll construct the SDK to pull exactly what is needed for controllers.

3. **Provide Compiled Libs in `SDK/lib`:**
   - The user noted that the executable will be created inside the repo as it usually does. Since the codebase seems complex and has many internal link paths, the request states: "we will make the files as you can see in cmake files then after the exe files has been created we only wanna keep sdk codes and those exe files... no the exe it self will be created... we dont wanna change that... this task is only about packaging the existing codebase into an sdk".
   - The user basically wants a generic, empty controller inside `user/` (like `IUST_Controller`) that exposes `q`, `qdot`, `tau`, `kp`, `kd`, which they can then package.
   - Ah wait, let's re-read the prompt: "we wanna write an SDK for this repo that we put in robot board so that any one wanting to run his own codes uses this SDK to run his controller and robot can do his job. finally we wanna make the robot codes and then we will delete the robot codes and only put the exe files of our own controller and the SDK files... we can have an script that the robot users can see them and use it to run their own controller and when they extract the q, qdot and tau and kp and kd... no this task is only about packaging the existing codebase into an sdk"
   - They just want a new generic controller inside the repo, exposing these parameters, maybe inside an `SDK` directory, and I don't need to actually write a script to package it - I just need to create the code layout for this "SDK" so it can be built and used.

4. **New generic controller (SDK/CustomController or user/CustomController):**
   - Create a directory `user/CustomController`.
   - Add a `CustomController.hpp`, `CustomController.cpp`, `main.cpp`, and `CustomUserParameters.h` inside it (modeled after `Example_Leg_InvDyn`).
   - In `CustomController.cpp`, clearly expose reading `q`, `qdot` from state, and setting `qDes`, `qdDes`, `tauFeedForward`, `kpJoint`, and `kdJoint` for `_legController->commands`.
   - Update `user/CMakeLists.txt` to include `add_subdirectory(CustomController)`.
   - I will also create an `sdk_export.sh` script (or `Makefile` wrapper) in the repo root as a convenience for them to package out the built exe and the necessary files if needed, or simply let them know they can do it.

5. **Pre-commit Checks**
