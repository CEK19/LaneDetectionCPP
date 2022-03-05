# LaneDetectionCPP
Improving FPS of Lane Detection Algorithm

**Prequesite:**
- **Reference:** <br/>
  1. Install OpenCV: https://vitux.com/opencv_ubuntu/
  2. Connect VSCode with OpenCV https://medium.com/analytics-vidhya/how-to-install-opencv-for-visual-studio-code-using-ubuntu-os-9398b2f32d53
  3. Fixed Error:
    -  https://answers.opencv.org/question/217959/libopencv_coreso31-cannot-open-shared-object-file-no-such-file-or-directory/
- **Environment:** Linux 20.04
- **FIRST THING FIRST:** `cd ~`
- **[1] INSTALL OPENCV FOR PYTHON & C++**
  - **Step 1: Install build tools:** </br>
  `$ sudo apt install build-essential cmake git pkg-config libgtk-3-dev \  
  libavcodec-dev libavformat-dev libswscale-dev libv4l-dev \  
  libxvidcore-dev libx264-dev libjpeg-dev libpng-dev libtiff-dev \  
  gfortran openexr libatlas-base-dev python3-dev python3-numpy \  
  libtbb2 libtbb-dev libdc1394-22-dev libopenexr-dev \  
  libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev` 
  - **Step 2: Clone OpenCV’s repositories** <br/>
  `$ mkdir ~/opencv_build && cd ~/opencv_build` <br/> 
  `$ git clone https://github.com/opencv/opencv.git` <br />
  `$ git clone https://github.com/opencv/opencv_contrib.git` 
  - **Step 3: Setup OpenCV build:** <br />
  `$ cd ~/opencv_build/opencv` <br /> 
  `$ mkdir -p build && cd build` <br />
  `$ cmake -D CMAKE_BUILD_TYPE=RELEASE \
  -D CMAKE_INSTALL_PREFIX=/usr/local \
  -D INSTALL_C_EXAMPLES=ON \
  -D INSTALL_PYTHON_EXAMPLES=ON \
  -D OPENCV_GENERATE_PKGCONFIG=ON \
  -D OPENCV_EXTRA_MODULES_PATH=~/opencv_build/opencv_contrib/modules \
  -D BUILD_EXAMPLES=ON ..`
  - **Step 4: Start a compilation:** <br/>
  `$ make -j8`
  - **Step 5:  Install OpenCV:** <br/>
  `$ sudo make install` <br/>
  After completing the installation process, type the following command to verify the OpenCV installation. For C++ binding: </br>
  `$ pkg-config --modversion opencv4` <br/>
  For python binding run the command as follows: <br/>
  `$ pkg-config --modversion opencv4` <br/>
- **[2] CONNECT VSCode with OpenCV**
  - **Step 1:** Open the visual studio code -> create a new folder (say “project”) -> create a new cpp file (say “new.cpp”) 
  - **Step 2:** Step2: Copy paste a picture (say “Lena.jpg”) into the new folder created.  Now our folder structure should look similar to the one below <br/>
  `project` <br/>
  `|--new.cpp` <br/>
  `|--Lena.png`
  - **Step 3:** Copy paste the below code and don’t forget to change the image name if required and also make sure to check if there are any syntax errors are present before moving further:
  ```
  #include <opencv2/highgui.hpp>
  #include <iostream>
  int main( int argc, char** argv )
  {
      cv::Mat image;
      image = cv::imread("Lena.png",cv::IMREAD_COLOR);
      if(! image.data)
          {
              std::cout<<"Could not open file" << std::endl;
              return -1;
          }
      cv::namedWindow("namba image", cv::WINDOW_AUTOSIZE);
      cv::imshow("namba image", image);
      cv::waitKey(0);
      return 0;
  }
  ```
  - **Step 4:** With the main.cpp file open press “ctrl+shift+p”, this will open the command pallet, In that select “C/C++: Edit     Configurations(JSON)” to open “c_cpp_properties.json” as shown below:
  - <img src="https://user-images.githubusercontent.com/69042380/151328012-ce0d4b10-fbe5-4eb9-b388-10ed120082bd.png" width="600" height="400" />
  - **Step 5:** In _“c_cpp_properties.json”_ edit the IncludePath to add _“/usr/local/include/opencv4/**”_ as shown below:
  ```
  {
    "configurations": [
        {
            "name": "Linux",
            "includePath": [
                "${workspaceFolder}/**",
                "/usr/local/include/opencv4/**"
            ],
            "defines": [],
            "compilerPath": "/usr/bin/gcc",
            "cStandard": "gnu17",
            "cppStandard": "gnu++14",
            "intelliSenseMode": "linux-gcc-x64"
        }
    ],
    "version": 4
  }  
  ```
  - **Step 6:** Now create another file and name it as “Makefile”, our folder structure should be something similar to what shown     below in the snippet:
  ```
  project
  |--new.cpp
  |--Lena.png
  |--Makefile
  ```
  Inside the Makefile type the below code. Good to type the code below instead of copy pasting since that may result in error.
  (Note: Make sure that the last line has a tab character at the beginning instead of four spaces while you type the command)
  ```
  CC = g++
  PROJECT = new_output
  SRC = new.cpp
  LIBS = `pkg-config --cflags --libs opencv4`
  $(PROJECT) : $(SRC)
      $(CC) $(SRC) -o $(PROJECT) $(LIBS)
  ```
  - **Step 7:** Now go to the terminal in the VS code and type “make”. This should give you an executable called “new_output”
  - **Step 8:** Now execute ./new_ouput. This should pop our image
- **[3] Some Error and Solution**
  1. `cannot open shared object file: No such file or directory` <br/>
  [SOLUTION]: Using `sudo ldconfig -v` before using `make`
  
 **Scope of this project**
 - With streets:
    - 2-way street
    - Turn left, turn right
    - Crossroad + T junction
    - Have dashed and curved line
  - With Obstacles that Lidar can detect:
    - Move with medium velocity (when comparing with Turtlebot) and straight direction.
  - With Obstacles that Lidar can't detect:
    - Can go through
    - Visible with Camera
    - Smaller than Turtlebot
    - Noise is not too much
