import sys
import time
import random
import string

from rapyuta_io import Client, ROSDistro, SimulationOptions, \
    BuildOptions, CatkinOption, Build, DeploymentPhaseConstants

AUTH_TOKEN = 'JxHZkPjMP1D78JbmDLGtvVJIy6fTtQXAVNPc3myE'
PROJECT_ID = 'project-zxfsnuxqpomamdcjgvznhccv'


def get_random_string(len):
    return ''.join(random.sample(string.lowercase, len))

def dep_polling_till_phase_succeed(dep):
    dep_status = None
    for i in range(15):
        print('polling ...')
        dep_status = dep.get_status()
        if dep_status.phase != DeploymentPhaseConstants.SUCCEEDED.value:
            time.sleep(6)
        else:
            break
    print('successfully provisioned {}'.format(dep.name))
    return dep_status

def create_build():
    build = Build(buildName='{}-{}'.format('fkie_benchmarking', get_random_string(2)),
                  strategyType='Source',
                  repository='https://github.com/smrutisenapati/fkie-benchmarking',
                  architecture='amd64',
                  rosDistro='kinetic',
                  isRos=True)
    print('creating build...')
    build = client.create_build(build)
    print('polling...')
    build.poll_build_till_ready()
    print('build created')
    return build

def create_routed_network():
    print('creating routed network...')
    routed_network = client.create_cloud_routed_network(
        'rn' + '-' + get_random_string(2), ROSDistro.KINETIC, True)
    print('polling...')
    routed_network.poll_routed_network_till_ready()
    print('routed network created')
    return routed_network

def create_native_network_package():
    print('creating native network package...')
    native_network_manifest = {
        "name": "nativenetwork" + "-" + get_random_string(2),
        "packageVersion": "v1.0.0",
        "description": "",
        "bindable": True,
        "plans": [
            {
                "name": "default",
                "metadata": {},
                "singleton": False,
                "components": [
                    {
                        "name": "nativenetwork",
                        "description": "",
                        "cloudInfra": {
                            "replicas": 1,
                            "endpoints": []
                        },
                        "ros": {
                            "topics": [
                                {
                                    "name": "/telemetry",
                                    "qos": "low",
                                    "compression": "",
                                    "scoped": False,
                                    "targeted": False
                                },
                                {
                                    "name": "/cmd_vel",
                                    "qos": "low",
                                    "compression": "",
                                    "scoped": False,
                                    "targeted": False
                                }
                            ],
                            "services": [],
                            "actions": [],
                            "isROS": True,
                            "ros_distro": "kinetic"
                        },
                        "requiredRuntime": "cloud",
                        "architecture": "amd64",
                        "executables": [
                            {
                                "name": "exec1",
                                "simulationOptions": {
                                    "simulation": False
                                },
                                "cmd": [
                                    "/bin/bash",
                                    "-c",
                                    "sleep infinity"
                                ],
                                "docker": "ubuntu:latest",
                                "limits": {
                                    "cpu": 0.1,
                                    "memory": 1024
                                }
                            }
                        ],
                        "parameters": [
                            {
                                "default": "hitesh",
                                "name": "NATIVE_NETWORK_INSTANCE",
                                "description": ""
                            }
                        ],
                        "rosBagJobDefs": []
                    }
                ],
                "includePackages": [],
                "dependentDeployments": [],
                "inboundROSInterfaces": {
                    "anyIncomingScopedOrTargetedRosConfig": False
                },
                "exposedParameters": []
            }
        ]
    }
    package_details = client.create_package(native_network_manifest)
    print('native network package created')
    return package_details

def create_publisher_package(build_guid, component_instance_id):
    print('creating publisher package...')
    publisher_manifest = {
        "name": "fkie-publisher"+ "-" + get_random_string(2),
        "packageVersion": "v1.0.0",
        "description": "",
        "bindable": True,
        "plans": [
            {
                "name": "default",
                "metadata": {},
                "singleton": False,
                "components": [
                    {
                        "name": "comp",
                        "description": "",
                        "cloudInfra": {
                            "replicas": 1,
                            "endpoints": []
                        },
                        "ros": {
                            "topics": [],
                            "services": [],
                            "actions": [],
                            "isROS": True,
                            "ros_distro": "kinetic"
                        },
                        "requiredRuntime": "cloud",
                        "architecture": "amd64",
                        "executables": [
                            {
                                "name": "exec",
                                "simulationOptions": {
                                    "simulation": False
                                },
                                "buildGUID": build_guid,
                                "cmd": [
                                    "roslaunch fkie_benchmarking talker.launch"
                                ],
                                "limits": {
                                    "cpu": 0.1,
                                    "memory": 1024
                                }
                            }
                        ],
                        "parameters": [
                            {
                                "default": component_instance_id,
                                "name": "NATIVE_NETWORK_INSTANCE",
                                "description": ""
                            }
                        ],
                        "rosBagJobDefs": []
                    }
                ],
                "includePackages": [],
                "dependentDeployments": [],
                "inboundROSInterfaces": {
                    "anyIncomingScopedOrTargetedRosConfig": False
                },
                "exposedParameters": []
            }
        ]
    }
    package_details = client.create_package(publisher_manifest)
    print('created publisher package...')
    return package_details

def deprovison_all_deps(pkg_id):
    pkg = client.get_package(pkg_id)
    print('deprovisioning all deployments of this package {}'.format(pkg.packageName))
    deps = pkg.deployments()
    for dep in deps:
        dep.deprovision()
        print('deprovisioned {}'.format(dep.name))

def provision_pkg(pkg_id, count, prefix, instance_id, routed_network_guid):
    print('provisioning {} package...'.format(prefix))
    pkg = client.get_package(pkg_id)
    plan_id = pkg.plans[0].planId
    dep_status = None
    for i in range(count):
        cfg = pkg.get_provision_configuration(plan_id)
        net = client.get_routed_network(routed_network_guid)
        cfg.add_routed_networks([net])
        if instance_id:
            cfg.add_parameter('comp', 'NATIVE_NETWORK_INSTANCE', instance_id)
        dep = pkg.provision(prefix + '-' + get_random_string(3), cfg)
        print('successfuly started provisioning {}'.format(dep.name))
        dep_status = dep_polling_till_phase_succeed(dep)
    print('provisioned {} package'.format(prefix))
    return dep_status

# RIO_CONFIG=config.json python fkie_benchmarking --all <num_of_deployment>
# RIO_CONFIG=config.json python fkie_benchmarking --provision <num_of_deployment>
# RIO_CONFIG=config.json python fkie_benchmarking --deprovision

# fill these for working of provision and deprovision
routed_network_guid = 'net-cpsrjmroeoksyqyjzgwdupgt'
native_network_package_id = 'pkg-gsuzyzpoicuplnsiiongtvaj'
publisher_package_id = 'pkg-wykxwsnlnhwbxdxmooksarxg'

if __name__ == '__main__':
    client = Client(auth_token=AUTH_TOKEN, project=PROJECT_ID)
    if sys.argv[1] == '--provision':
        print('provision started...')
        print('publisher_package_id: {} native_network_package_id:{}'.
              format(publisher_package_id, native_network_package_id))
        dep_status = provision_pkg(native_network_package_id, 1,
                                   'nativenetwork', None, routed_network_guid)
        instance_id = dep_status.componentInfo[0].componentInstanceID
        provision_pkg(publisher_package_id, int(sys.argv[2]),
                      'publisher', instance_id, routed_network_guid)
        print('native network component instance id: ' + instance_id)
    elif sys.argv[1] == '--deprovision':
        print('deprovision started...')
        print('publisher_package_id: {} native_network_package_id:{}'.
            format(publisher_package_id, native_network_package_id))
        deprovison_all_deps(publisher_package_id)
        deprovison_all_deps(native_network_package_id)
    elif sys.argv[1] == '--all':
        # creating build
        publisher_build = create_build()
        publisher_build_guid = publisher_build.guid
        # creating routed network
        routed_network = create_routed_network()
        routed_network_guid = routed_network.guid

        # creating native network package
        native_network_package_details = create_native_network_package()
        native_network_package_id = native_network_package_details['packageId']
        # provisioning native network
        dep_status = provision_pkg(native_network_package_id, 1,
                                   'nativenetwork', None, routed_network_guid)
        native_deployement_comp_instance_id = dep_status.componentInfo[0].componentInstanceID

        # create publisher package
        publisher_package_details = create_publisher_package(
            publisher_build_guid, native_deployement_comp_instance_id)
        publisher_package_id = publisher_package_details['packageId']
        # provisioning publisher package
        provision_pkg(publisher_package_id, int(sys.argv[2]), 'publisher',
                      native_deployement_comp_instance_id, routed_network_guid)

        print('native network component instance id: ' + native_deployement_comp_instance_id)
