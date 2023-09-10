# games101

## 环境配置

由于虚拟机的性能较低，且使用时存在诸多不便，我在原生系统中进行了环境配置。下面分别介绍了在 Windows 和 macOS 中的配置的方法。

### Windows

第一步，安装 vcpkg。

GitHub 链接： https://github.com/microsoft/vcpkg

将源码克隆到本地。

```
git clone https://github.com/microsoft/vcpkg.git
```

运行 bootstrap-vcpkg.bat 文件。执行完毕后，文件夹内会出现 vcpkg.exe 文件，此时便完成了 vcpkg 的安装。

第二步，安装需要的库。

以 Eigen 为例，打开 PowerShell，进入 vcpkg 所在文件夹，执行如下命令：

```
./vcpkg install eigen3:x64-windows
```

注意，需要安装 64 位版本。

第三步，集成到 Visual Studio 全局环境。执行如下命令：

```
./vcpkg integrate install
```

至此，便可以在 Visual Studio 中愉快地编写代码了。


### macOS

第一步，安装 Homebrew。

Homebrew 官网： https://brew.sh

打开终端，执行以下命令：

```
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
```

第二步，安装需要的库：

以 OpenCV 为例，执行如下命令：

```
brew install opencv
```

第三步，在 Xcode 中设置 Search Paths。

在左侧的项目导航栏中点击项目，在右侧点击一个 TARGET，在 Building Settings 中，找到 Search Paths 设置分类，将 `/opt/homebrew/include` 和 `/opt/homebrew/include/opencv4` 添加至 Header Search Paths，将 `/opt/homebrew/lib` 添加至 Library Search Paths。

对于 OpenCV，还需将 `/opt/homebrew/include/opencv4` 添加至 Header Search Paths，以及设置 Other Linker Flags。设置方法如下：

找到 `opencv.pc` 文件的路径。

以我的安装版本为例，该路径为 `/opt/homebrew/Cellar/opencv/4.8.0_3/lib/pkgconfig/opencv4.pc` 。

在终端执行以下命令查看 OpenCV 的 Linker Flags：

```
pkg-config --cflags --libs /opt/homebrew/Cellar/opencv/4.8.0_3/lib/pkgconfig/opencv4.pc
```

得到类似于下面的结果：

```
-I/opt/homebrew/opt/opencv/include/opencv4 -L/opt/homebrew/opt/opencv/lib -lopencv_gapi -lopencv_stitching -lopencv_alphamat -lopencv_aruco -lopencv_bgsegm -lopencv_bioinspired -lopencv_ccalib -lopencv_dnn_objdetect -lopencv_dnn_superres -lopencv_dpm -lopencv_face -lopencv_freetype -lopencv_fuzzy -lopencv_hfs -lopencv_img_hash -lopencv_intensity_transform -lopencv_line_descriptor -lopencv_mcc -lopencv_quality -lopencv_rapid -lopencv_reg -lopencv_rgbd -lopencv_saliency -lopencv_sfm -lopencv_stereo -lopencv_structured_light -lopencv_phase_unwrapping -lopencv_superres -lopencv_optflow -lopencv_surface_matching -lopencv_tracking -lopencv_highgui -lopencv_datasets -lopencv_text -lopencv_plot -lopencv_videostab -lopencv_videoio -lopencv_viz -lopencv_wechat_qrcode -lopencv_xfeatures2d -lopencv_shape -lopencv_ml -lopencv_ximgproc -lopencv_video -lopencv_xobjdetect -lopencv_objdetect -lopencv_calib3d -lopencv_imgcodecs -lopencv_features2d -lopencv_dnn -lopencv_flann -lopencv_xphoto -lopencv_photo -lopencv_imgproc -lopencv_core
```

将该结果复制到 Xcode 中 Building Settings 下的 Other Linker Flags 中。

至此，便可以在 Xcode 中愉快地编写代码了。
