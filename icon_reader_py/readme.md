# Python iCon reader

Python script for loading iCon files from disk. 

## Dependencies

Run `pip install -r requirements.txt`

## Examples

The examples folder contains three examples of how to use the icon reader. 

### Plot image
Reads the image specified in input argument and plots its components and mark data. 

`python3 icon_plotter.py icon_xml_path`

<img src="docs/img/icon_plotter.png" width="100%">

### Convert to PLY
Reads all images in a folder and converts them to PLY format. The input argument -i can be set either to a folder path to convert all icon images in that folder, or to an xml file to convert one specific image. 

`python3 icon_to_ply_converter.py -o output_folder -i input_folder`

### Convert to PCD
Reads all images in a folder and converts them to PCD format. The input argument -i can be set either to a folder path to convert all icon images in that folder, or to an xml file to convert one specific image.

`python3 icon_to_pcd_converter.py -o output_folder -i input_folder`

### Convert to AppSpace compatible json
Reads all images in a folder and converts them to AppSpace compatible .json format. The input argument -i can be set either to a folder path to convert all icon images in that folder, or to an xml file to convert one specific image. 

`python3 icon_to_json_converter.py -o output_folder -i input_folder`

<img src="docs/img/icon_to_appspace_json.png" width="95%">

