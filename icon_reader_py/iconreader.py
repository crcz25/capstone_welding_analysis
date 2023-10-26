from enum import Enum
from typing import Dict
import xml.etree.ElementTree as ET
import os
import numpy as np
from collections import defaultdict


class SubcomponentType(Enum):
    """
    Enum for subcomponent types
    """
    RANGE = "Range"
    RANGE_X = "Range X"
    INTENSITY = "Intensity"
    SCATTER = "Scatter"
    MARK = "Mark"
    SENSOR_IMAGE = "Image"
    UNKNOWN = "Unknown"

    def string_to_type(type_str : str):
        types = {
            "range": SubcomponentType.RANGE,
            "range r": SubcomponentType.RANGE,
            "range x": SubcomponentType.RANGE_X,
            "intensity": SubcomponentType.INTENSITY,
            "reflectance": SubcomponentType.INTENSITY,
            "scatter": SubcomponentType.SCATTER,
            "mark": SubcomponentType.MARK,
            "image": SubcomponentType.SENSOR_IMAGE
        }
        if type_str.lower() in types:
            return types[type_str.lower()]
        else:
            return SubcomponentType.UNKNOWN

class Subcomponent:
    """Contains the properties and data of a subcomponent.

    Attributes
    ----------
    subcomponent_type : SubcomponentType
        Type of subcomponent
    name : str
        Name of the subcomponent
    data : np.ndarray
        Raw data, from .dat file
    size : int
        Size of the subcomponent
    width : int
        Width of the subcomponent
    dtype : np.dtype
        Subcomponent's data type

    """
    def __init__(self, subcomponent_type : SubcomponentType, subcomponent_xml : str):
        name, dtype, size, width, = self.__parse_xml(subcomponent_xml)
        self.subcomponent_type = subcomponent_type
        self.name = name
        self.data = None
        self.size = size
        self.width = width
        self.dtype = np.dtype(dtype)

    def __parse_xml(self, subcomponent_xml):
        name = subcomponent_xml
        value_type_str = subcomponent_xml.get("valuetype")
        str_to_type = {
            "WORD": np.uint16,
            "BYTE": np.uint8,
            "INT": np.int32,
            "FLOAT": np.float32,
        }
        value_type = str_to_type[value_type_str]
        size = None
        width = None
        for param in subcomponent_xml.findall("parameter"):
            if param.get("name") == "size":
                size = int(param.text)
            elif param.get("name") == "width":
                width = int(param.text)

        return name, value_type, size, width

    def __str__(self):
        return f"{self.subcomponent_type}, width: {self.width}, size: {self.size}"

    def __repr__(self):
        return str(self)

class Traits:
    """
    A class representing traits for one component.

    Attributes
    ----------
    size : int
        Size of the component
    world_range_traits : dict
        Dictionary with all world range traits, parsed from a component. Is None if the component does not contain a worldrangetraits element.
    sensor_range_traits
        Dictionary with all sensor range traits, parsed from a component. Is None if the component does not contain a sensorrangetraits element.
    genistream_traits
        Dictionary with all genistream traits, parsed from a component. Is None if the component does not contain a genistreamtraits element.
    image_traits
        Dictionary with all image traits, parsed from a component. Is None if the component does not contain a imagetraits element.

    Methods
    -------
    has_world_range_traits()
        Returns True if world range traits exists
    has_sensor_range_traits()
        Returns True if sensor range traits exists
    has_genistream_traits()
        Returns True if genistream traits exists
    has_image_traits()
        Returns True if image traits exists
    """
    def __init__(self, component_xml : str):
        self.size = self.__get_size(component_xml)
        self.world_range_traits = self.__parse_traits("worldrangetraits", component_xml)
        self.sensor_range_traits = self.__parse_traits("sensorrangetraits", component_xml)
        self.genistream_traits = self.__parse_traits("genistreamtraits", component_xml)
        self.image_traits = self.__parse_traits("imagetraits", component_xml)

    def __get_size(self, component_xml):
        for param in component_xml.findall("parameter"):
            if param.get("name") == "size":
                return int(param.text)
        raise ValueError("Image XML contains no size parameter")

    def __parse_traits(self, key, component_xml):
        traits_xml = component_xml.find(key)
        if traits_xml is None:
            return None

        traits = {}
        for param in traits_xml.findall("parameter"):
            traits[param.get("name")] = self.__parse_param(param.text if param.text is not None else "")

        return traits

    def __parse_param(self, value_str):
        if value_str.isnumeric():
            return int(value_str)
        elif self.__is_float(value_str):
            return float(value_str)
        elif value_str.lower() in ("true", "false"):
            return value_str == "true"
        else:
            return value_str

    def __is_float(self, value_str):
        try:
            float(value_str)
            return True
        except ValueError:
            return False

    def has_world_range_traits(self):
        """Returns True if world range traits exists"""
        return self.world_range_traits is not None

    def has_sensor_range_traits(self):
        """Returns True if sensor range traits exists """
        return self.sensor_range_traits is not None

    def has_genistream_traits(self):
        """Returns True if genistream traits exists"""
        return self.genistream_traits is not None

    def has_image_traits(self):
        """Returns True if image traits exists. """
        return self.image_traits is not None

class Component:
    """
    A class representing an image component.

    Attributes
    ----------
    traits :  Traits
    subcomponents : dict(SubcomponentType, Subcomponent)

    Methods
    -------
    get_range()
        Returns the data from the range subcomponent.
    get_intensity()
        Returns the data from the intensity subcomponent.
    get_scatter()
        Returns the data from the scatter subcomponent.
    get_mark()
        Returns the data from the mark subcomponent.
    get_sensor_image()
        Returns the data from the sensor image subcomponent.
    rescale_range_16bit_to_float()
        Rescale range data from uint16 to float
    rescale_range_float_to_16bit()
        Rescale range data from float to uint16
    """

    def __init__(self, name: str, traits : Traits, subcomponents : Subcomponent, x_positions : np.ndarray):
        self.name = name
        self.traits = traits
        self.subcomponents = subcomponents
        self.x_positions = x_positions

    def get_range(self) -> np.ndarray:
        """
        Returns the data from the range subcomponent.
        Requires the range subcomponent to be named 'Range' or 'Range R'.

        Returns
        -------
        range_image :
            Range image data
        """
        range_image = None
        if "Range" in self.subcomponents:
            range_image = self.subcomponents["Range"].data
        elif "Range R" in self.subcomponents:
            range_image = self.subcomponents["Range R"].data
        return range_image

    def get_intensity(self) -> np.ndarray:
        """
        Returns the data from the intensity subcomponent.
        Requires the intensity subcomponent to be named 'Intensity' or 'Reflectance'.

        Returns
        -------
        intensity_image :
            Intensity image data
        """
        intensity_image = None
        if "Intensity" in self.subcomponents:
            intensity_image = self.subcomponents["Intensity"].data
        elif "Reflectance" in self.subcomponents:
            intensity_image = self.subcomponents["Reflectance"].data
        return intensity_image

    def get_scatter(self) -> np.ndarray:
        """
        Returns the data from the scatter subcomponent.
        Requires the scatter subcomponent to be named 'Scatter'.

        Returns
        -------
        scatter_image :
            Scatter image data
        """
        scatter_image = None
        if "Scatter" in self.subcomponents:
            scatter_image = self.subcomponents["Scatter"].data
        return scatter_image

    def get_mark(self) -> np.ndarray:
        """
        Returns the data from the mark subcomponent.
        Requires the mark subcomponent to be named 'Mark'.

        Returns
        -------
        mark :
            Mark data
        """
        mark = None
        if "Mark" in self.subcomponents:
            mark = self.subcomponents["Mark"].data
        return mark

    def get_sensor_image(self) -> np.ndarray:
        """
        Returns the data from the sensor image subcomponent.
        Requires the sensor image subcomponent to be named 'Image'.

        Returns
        -------
        sensor_image :
            Sensor image data
        """
        sensor_image = None
        if "Image" in self.subcomponents:
            sensor_image = self.subcomponents["Image"].data
        return sensor_image

    def rescale_range_16bit_to_float(self) -> np.ndarray:
        """
        Rescale range data from uint16 to float

        Returns
        -------
        data_float :
            Range data of type float
        """
        world_range_traits = self.traits.world_range_traits
        z_min = world_range_traits["lower bound r"]
        z_max = world_range_traits["upper bound r"]
        data_16 = self.get_range()

        data_float = ((((data_16 - 1) / (2 ** 16 - 2)) * (z_max - z_min)) + z_min).astype(np.float32)
        data_float[data_16 == 0] = 0.0
        return data_float

    def rescale_range_float_to_16bit(self) -> np.ndarray:
        """
        Rescale range data from float to uint16

        Returns
        -------
        data_16 :
            Range data of type uint16
        """
        world_range_traits = self.traits.world_range_traits
        z_min = world_range_traits["lower bound r"]
        z_max = world_range_traits["upper bound r"]
        data_float = self.get_range()
        data_16 = (((data_float - z_min) / (z_max - z_min)) * (2 ** 16 - 2) + 1).astype(np.uint16)
        data_16[data_float == 0.0] = 0

        return data_16

class Mark:
    """A class used to represent mark data.

    Attributes
    ----------
    encoder_value : int
    frame_trigger_active : bool
    encoder_a : bool
    encoder_b : bool
    line_trigger_active : bool
    encoder_reset_active : bool
    overtrigger_count : int
    sample_timestamp : int
    """
    def __init__(self, data):
        _, mark_width = data.shape
        self.encoder_value = data[:, 0]
        io_status = data[:, 1]

        self.frame_trigger_active = (io_status & (1 << 30)) > 0  # bit 30
        self.encoder_a = (io_status & (1 << 28)) > 0  # bit 28
        self.encoder_b = (io_status & (1 << 27)) > 0  # bit 27
        self.line_trigger_active = (io_status & (1 << 25)) > 0  # bit 25
        self.encoder_reset_active = (io_status & (1 << 24)) > 0  # bit 24
        self.overtrigger_count = (io_status & 0x00FF0000) >> 16

        self.sample_timestamp = None
        if mark_width == 5:
            self.sample_timestamp = data[:, 2].astype(np.uint32)


class IConReader:
    """Reads Icon (xml / dat) files.

    Attributes
    ----------
    xml_path : str
        Path to icon .xml file
    dat_path :str
        Path to icon .dat file
    available_components : dict(str, dict(str, Subcomponent))
    is_sensor_image : bool
        True if image is a sensor image
    traits : dict(str, Trait)
        Dictionary with each component's traits, read from the xml
    height :
        Height of the first subcomponent in the image

    Methods
    -------
    load_image_components(parse_mark=True)
        Loads all data from the .dat and stores it in Subcomponents.
    get_coordinate_system_traits(component, y_res=1.0)
        Returns the pizel size and origin of the input component.

    """

    def __init__(self, xml_path : str):
        """
        Parameters
        ----------
        xml_path : str
            Path to the icon .xml file to load
        """
        self.xml_path = os.path.abspath(xml_path)
        self.dat_path = os.path.splitext(self.xml_path)[0] + ".dat"
        tree = ET.parse(xml_path)
        root = tree.getroot()
        if root.tag != 'icon_data_format':
            raise ValueError('Invalid XML format')
        self.available_components, self.is_sensor_image = self.__get_subcomponent_properties(root)
        self.traits = self.__get_traits(root)
        self.height = self.__get_height(root, self.is_sensor_image)

    def load_image_components(self, parse_mark=True ) -> Dict[str, Component]:
        """
        Loads all data from the .dat and stores it in Subcomponents.

        Parameters
        ----------
        parse_mark : bool
            Flag used to parse the mark data (default is True)

        Returns
        -------
        components : dict
            Returns a dictionary with components, where each Component has its subcomponents and traits.
        """
        buffer = None
        with open(self.xml_path[:-4] + '.dat', 'rb') as file:
            buffer = file.read()
            print(f"Read {len(buffer)} bytes from {self.xml_path[:-4] + '.dat'}")

        components = {}
        x_positions = None
        offset = 0

        for name, component in self.available_components.items():
            component_traits = self.traits[name]

            for subcomponent_type, subcomponent in component.items():
                value_type = subcomponent.dtype
                width = subcomponent.width
                data, num_bytes_read = self.__get_data(buffer, width, self.height, offset, value_type)
                offset += num_bytes_read
                if subcomponent_type == SubcomponentType.MARK.value and parse_mark:
                    subcomponent.data = Mark(data)
                elif subcomponent_type == SubcomponentType.RANGE_X.value:
                    x_positions = data
                else:
                    subcomponent.data = data

            result_component = Component(name, component_traits, component, x_positions)

            if x_positions is None and result_component.get_range() is not None:
                result_component.x_positions = self.__generate_x_positions(result_component.get_range(), name)

            components[name] = result_component

        return components

    def get_coordinate_system_traits(self, component, y_res=1.0):
        """
        Returns the pizel size and origin of the input component.
        """

        world_range_traits = self.traits[component.name].world_range_traits
        sensor_range_traits = self.traits[component.name].sensor_range_traits
        genistream_traits = self.traits[component.name].genistream_traits

        pixel_size = {}
        pixel_size["y"] = genistream_traits["b axis range scale"] if genistream_traits is not None else y_res
        pixel_size["x"] = self.__get_x_scale(component.name)
        pixel_size["z"] = self.__get_z_scale(component.name)

        origin = {}
        origin["y"] = 0
        if world_range_traits is not None:
            origin["x"] = world_range_traits["lower bound x"]
            origin["z"] = 0.0
        else:
            origin["x"] = sensor_range_traits["origin x"]
            origin["z"] = 0

        return pixel_size, origin

    def __get_data(self, buffer, width, height, offset, dtype):
        item_size = np.dtype(dtype).itemsize
        data = np.frombuffer(
            buffer,
            dtype,
            width *
            height,
            offset)

        image = np.reshape(data, (height, width))
        num_bytes_read = height * width * item_size

        return image, num_bytes_read

    def __get_z_scale(self, component_name):
        range_dtype = self.available_components[component_name][SubcomponentType.RANGE.value].dtype

        scale_z = None

        if range_dtype == np.float32:
            return 1.0

        world_range_traits = self.traits[component_name].world_range_traits
        sensor_range_traits = self.traits[component_name].sensor_range_traits

        if world_range_traits is not None:
            scale_z = (world_range_traits["upper bound r"] - world_range_traits["lower bound r"]) / (2 ** 16 - 2)

        elif sensor_range_traits is not None:
            scale_z = np.abs(sensor_range_traits["scale z"])

        return scale_z

    def __get_x_scale(self, component_name):
        world_range_traits = self.traits[component_name].world_range_traits

        if world_range_traits is not None:
            width = self.available_components[component_name][SubcomponentType.RANGE.value].width
            lower_bound_x = world_range_traits["lower bound x"]
            upper_bound_x = world_range_traits["upper bound x"]
            return (upper_bound_x - lower_bound_x) / width
        else:
            return 1.0

    def __generate_x_positions(self, range_image, component_name):
        height, width = range_image.shape
        data_x = np.empty_like(range_image, np.float64)
        x_scale = self.__get_x_scale(component_name)
        arr = np.linspace(0, width - 1, width) * x_scale
        for j in range(height):
            data_x[j][:] = arr
        return data_x

    def __get_traits(self, xml_root):
        traits = {}
        for component_xml in xml_root.iter('component'):
            name = component_xml.get("name")
            traits[name] = Traits(component_xml)
        return traits

    def __get_subcomponent_properties(self, xml_root):
        is_sensor_image = False
        components = defaultdict(dict)
        for component in xml_root.iter('component'):
            component_name = component.get("name")
            for subcomponent in component.findall('subcomponent'):
                subcomponent_type = SubcomponentType.string_to_type(subcomponent.get("name"))
                subcomponent_name = subcomponent.get("name")
                is_sensor_image = is_sensor_image or subcomponent_type == SubcomponentType.SENSOR_IMAGE
                components[component_name][subcomponent_name] = Subcomponent(subcomponent_type, subcomponent)
        return components, is_sensor_image

    def __get_height(self, root, is_sensor_image):
        height = None
        if is_sensor_image:
            for component_xml in root.iter('component'):
                for param in component_xml.findall('parameter'):
                    if param.get('name') == 'height':
                        height = int(param.text)
        else:
            total_size = os.path.getsize(self.dat_path)
            local_size = 0
            for param in root.findall('parameter'):
                if param.get('name') == 'size':
                    local_size = int(param.text)
                    break

            height = total_size // local_size

        return height
