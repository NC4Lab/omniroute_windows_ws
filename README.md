# omniroute_windows_ws folder structure

omniroute_windows_ws/               
├── build/
├── devel/
├── src/                             
│   └── projection_calibration/      
│       ├── src/                     
│       │   ├── projection_calibration.cpp
│       │   └── projection_calibration.h
│       └── CMakeLists.txt
└── lib/                            
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
    │   ├── lib/
    │   │   └── glfw3.lib
    │   └── bin/
    │       └── glfw3.dll
    ├── glm/  
    │       └── <header_files>.hpp
    ├── pugixml/  
    │       └── pugixml.cpp
    │       └── pugixml.hpp
    │       └── pugiconfig.hpp
    └── DevIL/
        ├── include/
        │   └── IL/
        │       ├── il.h
        │       ├── ilu.h
        │       ├── ilut.h
        │       └── devil_cpp_wrapper.h
        ├── lib/
        │   ├── DevIL.lib
        │   ├── ILU.lib
        │   └── ILUT.lib
        └── bin/                    
            ├── DevIL.dll
            ├── ILU.dll
            └── ILUT.dll
