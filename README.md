# CamVox data extraction

This is a ros package for extracting dataset provided by [CamVox](https://github.com/ISEE-Technology/CamVox) from rosbag.

## Current feature

* Extracting RGB image
* Extracing depth image

## Usage

```shell
rosrun camvox_data_extraction <path-to-rosbag> <path-to-extraction-dir>
```

The desired structure of the extraction directory looks like the following:

```
camvox_dataset/
  |-- color_img/
  |-- depth_img/
```

Make sure you have manually created the directory structure before running the node.

Note that `<path-to-extraction-dir>` should not include the last '/'. An example path: `/foo/bar/camvox_dataset`.