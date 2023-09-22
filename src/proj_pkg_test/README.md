# proj_pkg_test

### ==================== INSTALL GLAD LIBRARY ====================

1. **Download GLAD**
    - Visit the [GLAD Generator](https://glad.dav1d.de/) website.
    - Choose the language as `C/C++`.
    - For the API, select `OpenGL` and the version you need (e.g., 4.6).
    - Click `Generate` and download the zip file.

2. **Copy Headers and Source Files**
    - Extract the downloaded zip file and copy the `include` and `src` folders from the extracted folder to your project directory or a common libraries directory.

3. **Set Environment Variable**
    - Open a Command Prompt as an administrator and execute the following command to set the `GLAD_DIR` environment variable. Replace (Example) `C:\Program Files (x86)\OmnirouteSharedLibs\glad` with the actual path where you have saved the GLAD library.

    ```cmd
    setx GLAD_DIR "C:\Program Files (x86)\OmnirouteSharedLibs\glad" /M
    ```
    - The `/M` option sets the variable as a System variable (for all users).


### ==================== INSTALL GLFW LIBRARY ====================

1. **Download GLFW**
    - Visit the [GLFW Download Page](https://www.glfw.org/download.html).
    - Download the latest version of the source package or the pre-compiled Windows binaries, depending on your needs.

2. **Copy Headers and Library Files**
    - Extract the downloaded zip file and Copy the `include` folder and the contents of the `lib-vc2019` (or relevant `lib` folder depending on your Visual Studio version) to your project directory or a common libraries directory.

3. **Set Environment Variable**
    - Open a Command Prompt as an administrator and execute the following command to set the `GLFW_DIR` environment variable. Replace (Example) `C:\Program Files (x86)\OmnirouteSharedLibs\glfw` with the actual path where you have saved the GLFW library.
    ```cmd
    setx GLFW_DIR "C:\Program Files (x86)\OmnirouteSharedLibs\glfw" /M
    ```
    - The `/M` option sets the variable as a System variable (for all users).


## ==================== DELETING AN ENVIRONMENT VARIABLE ON WINDOWS 10 ====================

If you need to delete an environment variable that you've set earlier, you can do so using the Command Prompt to delete the system-wide variable. You'll need administrative privileges to delete it. Open a Command Prompt as an administrator and run:
    
    ```cmd
    reg delete "HKLM\\SYSTEM\\CurrentControlSet\\Control\\Session Manager\\Environment" /F /V VARIABLE_NAME
    ```

After deleting the variable, you may need to restart any open Command Prompt windows or applications to update their environment.

## ==================== EXAMPLE LIBRARY FOLDER STRUCTURE ====================

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