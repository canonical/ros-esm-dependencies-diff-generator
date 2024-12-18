import argparse
from pathlib import Path
import yaml

import rosdistro
from catkin_pkg.package import parse_package
from catkin_pkg.packages import find_package_paths
from rosdistro.dependency_walker import DependencyWalker
from rosinstall_generator.distro import generate_rosinstall


def main():
    parser = argparse.ArgumentParser(
        description="Generate a rosinstall with all the recursive build (buid, buildtool, build_export, buildtool_export), exec (exec, run) or test dependencies"
    )
    parser.add_argument(
        "--rosdistro",
        type=str,
        required=True,
        choices=("kinetic", "melodic", "noetic", "foxy", "humble", "jazzy"),
        help="The ROS distro to evaluate.",
    )
    parser.add_argument(
        "--depend-types",
        nargs="+",
        default=[
            "build",
            "build_export",
            "buildtool",
            "buildtool_export",
            "exec",
            "run",
            "test",
        ],
        help="A list of the depend types to list.",
    )
    parser.add_argument(
        "-o",
        "--output-file",
        type=str,
        required=True,
        help="The .rosinstall file to write.",
    )
    parser.add_argument(
        "-s",
        "--source",
        type=str,
        required=True,
        help="Source directory to look for packages.",
    )

    parser.add_argument(
        "--reference-ros-index-url",
        type=str,
        default="https://raw.githubusercontent.com/ros/rosdistro/master/index-v4.yaml",
        help="The packages reference ROS index URL.",
    )

    parser.add_argument(
        "--esm-ros-index-url",
        type=str,
        default="https://ros.robotics.ubuntu.com/rosdistros/index-v4.yaml",
        help="The ESM packages reference ROS index URL.",
    )

    parser.add_argument(
        "--ignore-packages",
        nargs="+",
        default=[
            "connext_cmake_module",
            "cyclonedds",
            "rmw_connext_cpp",
            "rmw_connext_shared_cpp",
            "rmw_cyclonedds_cpp",
            "rosidl_typesupport_connext_c",
            "rosidl_typesupport_connext_cpp",
        ],
        help="A list of packages to ignore in the dependencies."
        "The default is: ignore alternative DDS implementation packages.",
    )

    args = parser.parse_args()

    index = rosdistro.get_index(args.reference_ros_index_url)
    esm_index = rosdistro.get_index(args.esm_ros_index_url)

    esm_cache = rosdistro.get_distribution_cache(esm_index, args.rosdistro)
    ref_cache = rosdistro.get_distribution_cache(index, args.rosdistro)

    upstream_distribution = rosdistro.get_cached_distribution(
        index, args.rosdistro, cache=ref_cache
    )

    dependency_walker = DependencyWalker(upstream_distribution)

    source_packages_xml = find_package_paths(args.source)

    dependencies = set()
    source_packages = set()
    for package_xml in source_packages_xml:
        source_package = parse_package(Path(args.source) / Path(package_xml))
        source_packages.add(source_package.name)
        recursive_dependencies = dependency_walker.get_recursive_depends(
            source_package.name,
            args.depend_types,
            ros_packages_only=True,
            ignore_pkgs=args.ignore_packages,
        )
        dependencies.update(recursive_dependencies)

    system_dependencies = set(esm_cache.distribution_file.release_packages.keys())

    missing_source_dependencies = dependencies.difference(
        system_dependencies
    ).difference(source_packages)

    print(f"Found {len(missing_source_dependencies)} source to add.")
    rosinstall_content = generate_rosinstall(
        upstream_distribution, missing_source_dependencies
    )

    with open(args.output_file, "w") as yaml_file:
        yaml.safe_dump(rosinstall_content, yaml_file, default_flow_style=False)


if __name__ == "__main__":
    main()
