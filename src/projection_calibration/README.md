# proj_pkg_test

### ==================== INSTALL GLAD LIBRARY ====================

1. **Download GLAD**
    - Visit the [GLAD Generator](https://glad.dav1d.de/) website.
    - Language: `C/C++`.
    - Specification: `OpenGL`
    - APIs: `gl` 
    -   Version: `4.6` 
    - Profile: `Compatibility`
    - Click `Generate` and download the zip file.
2. **Copy Headers and Source Files**
    - Extract the downloaded zip file and copy the `include` and `src` folders from the extracted folder to your project directory or a common libraries directory.
    - Note the expected directory structure for the library below. 

3. **Set Environment Variable**
    - Open a Command Prompt as an administrator and execute the following command to set the `GLAD_DIR` environment variable. Replace (Example) `C:/Program Files (x86)/OmnirouteSharedLibs/glad` with the actual path where you have saved the GLAD library.
    - Note the use of foward slashes, which are necesary for CMake.
    - The `/M` option sets the variable as a System variable (for all users).

    ```cmd
    setx GLAD_DIR "C:/Program Files (x86)/OmnirouteSharedLibs/glad" /M
    ```

4. **Verification Environment Variable Created**
    - Close and reopen the Command Prompt as an administrator.
    
    ```cmd
    echo %GLAD_DIR%
    ```


### ==================== INSTALL GLFW LIBRARY ====================

1. **Download GLFW**
    - Visit the [GLFW Download Page](https://www.glfw.org/download.html).
    - Download the latest version of the source package or the pre-compiled Windows binaries, depending on your needs.
    - Suggested, download the '64-bit Windows binaries' option from the official site.

2. **Copy Headers and Library Files**
    - Extract the downloaded zip file and Copy the `include` folder and the contents of the `lib-vc2019` (or relevant `lib` folder depending on your Visual Studio version) to your project directory or a common libraries directory.
    - Note make sure to copy the `lib-vc2019` version and keep this folder name.
    - Note the expected directory structure for the library below.

3. **Set Environment Variable**
    - Open a Command Prompt as an administrator and execute the following command to set the `GLFW_DIR` environment variable. Replace (Example) `C:/Program Files (x86)/OmnirouteSharedLibs/glfw` with the actual path where you have saved the GLFW library.
    - Note the use of foward slashes, which are necesary for CMake.
    - The `/M` option sets the variable as a System variable (for all users).
    
    ```cmd
    setx GLFW_DIR "C:/Program Files (x86)/OmnirouteSharedLibs/glfw" /M
    ```

4. **Verification Environment Variable Created**
    - Close and reopen the Command Prompt as an administrator.
    
    ```cmd
    echo %GLFW_DIR%
    ```

### ==================== INSTALL DevIL LIBRARY ====================

1. **Download DevIL**
    - Visit the [DevIL GitHub Repository](https://github.com/DentonW/DevIL) or the [official website](http://openil.sourceforge.net/).
    - Download the source code or pre-compiled binaries suitable for your system.
    - Download 

2. **Copy Headers and Library Files**
    - Extract the downloaded zip file and copy the `include` and `lib` folders to a common libraries directory.
    - Note the expected directory structure for the library below.
    - Suggested, download the 'DevIL 1.8.0 SDK for Windows (3.01 MB)' option from the official site.

3. **Set Environment Variable**
    - Open a Command Prompt as an administrator and execute the following command to set the `DevIL_DIR` environment variable. Replace `(Example) C:/Program Files (x86)/OmnirouteSharedLibs/DevIL` with the actual path where you have saved the DevIL library.
    - Note the use of forward slashes, which are necessary for CMake.
    - The `/M` option sets the variable as a System variable (for all users).

    ```cmd
    setx DevIL_DIR "C:/Program Files (x86)/OmnirouteSharedLibs/DevIL" /M
    ```

4. **Verification Environment Variable Created**
    - Close and reopen the Command Prompt as an administrator.

    ```cmd
    echo %DevIL_DIR%
    ```


### ==================== SETUP ROS NOETIC ENVIRONMENT VARIABLE ====================

> **Note**: The Pugixml and OpenGL library are already included in the ROS Noetic distribution. The following steps are to ensure that the library is correctly linked in the project's CMakeLists.txt file without hardcoding any paths, making the workspace portable.

1. **Locate Pugixml in ROS Noetic**
    - Navigate to your ROS Noetic installation directory. Typically, it should be in `C:\opt\ros\noetic\x64`.

2. **Set Environment Variable**
    - Open a Command Prompt as an administrator and execute the following command to set the `ROS_NOETIC_DIR` environment variable. Replace `C:/opt/ros/noetic/x64` with the actual path where ROS Noetic is installed.
    - Note the use of forward slashes, which are necessary for CMake.
    - The `/M` option sets the variable as a System variable (for all users).

    ```cmd
    setx ROS_NOETIC_DIR "C:/opt/ros/noetic/x64" /M
    ```

3. **Verification Environment Variable Created**
    - Close and reopen the Command Prompt as an administrator.

    ```cmd
    echo %ROS_NOETIC_DIR%
    ```


## ==================== DELETING AN ENVIRONMENT VARIABLE ON WINDOWS 10 ====================

If you need to delete an environment variable that you've set earlier, you can do so using the Command Prompt to delete the system-wide variable. You'll need administrative privileges to delete it. Open a Command Prompt as an administrator and run:
    
    ```cmd
    reg delete "HKLM\SYSTEM\CurrentControlSet\Control\Session Manager\Environment" /F /V VARIABLE_NAME
    ```

After deleting the variable, you may need to restart any open Command Prompt windows or applications to update their environment.


## ==================== REQUIRED LIBRARY FOLDER STRUCTURE ====================

<external_lib_folder>/
├── glad/
│   ├── include/
│   │   └── glad/
│   │       └── glad.h
│   └── src/
│       └── glad.c
├── glfw/
│   ├── include/
│   │   └── GLFW/
│   │       └── glfw3.h
│   └── lib-vc2019/
│       └── glfw3.lib
│       └── glfw3.dll
└── DevIL/
    ├── include/
    │   └── IL/
    └── lib/
        └── x64
                └── Release
                        └── DevIL.dll
                        └── DevIL.lib
                        └── ILU.dll
                        └── ILU.lib
                        └── ILUT.dll
                        └── ILUT.lib

<ros_noetic_distrebution>/                            
├── x64/
│   ├── include/
│   │   └── pugixml.hpp
│   └── Lib/
│       └── pugixml.lib
│   └── bin/
│       └── pugixml.dll