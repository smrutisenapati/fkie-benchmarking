import sys
import time
import random
import string
import json

from rapyuta_io import Client, ROSDistro, SimulationOptions, \
    BuildOptions, CatkinOption, Build, DeploymentPhaseConstants

from rapyuta_io.clients.native_network import NativeNetwork, Limits, Parameters
from rapyuta_io.clients.package import Runtime, ROSDistro, RestartPolicy

AUTH_TOKEN = 'JxHZkPjMP1D78JbmDLGtvVJIy6fTtQXAVNPc3myE'
PROJECT_ID = 'project-zxfsnuxqpomamdcjgvznhccv'

# RIO_CONFIG=config.json python fkie_sdk --all <num_of_deployment>
# RIO_CONFIG=config.json python fkie_sdk --provision <num_of_deployment>
# RIO_CONFIG=config.json python fkie_sdk --deprovision

# fill these for working of provision and deprovision
native_network_guid = 'net-ksmcsdpwqdbezylelxvzbqow'
# native_network_package_id = 'pkg-dhjbspopnpngnkklsjcpauhv'
package_ids = []
publisher_build_guid = 'build-vpoiegzmgtrmzcoekiwpgsql'

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

def create_build(rosDistro='kinetic'):
    build = Build(buildName='{}-{}'.format('fkie_benchmarking', get_random_string(2)),
                  strategyType='Source',
                  repository='https://github.com/smrutisenapati/fkie-benchmarking',
                  architecture='amd64',
                  rosDistro=rosDistro,
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

def create_native_network(rosDistro='kinetic'):
    print('creating native network...')
    parameters = Parameters(Limits(1, 4096))
    native_network = NativeNetwork('nn' + '-' + get_random_string(2), Runtime.CLOUD, rosDistro,
                                   parameters=parameters)
    native_network = client.create_native_network(native_network)
    print('polling...')
    native_network.poll_native_network_till_ready()
    print('native network created')
    return native_network

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

def create_publisher_package(build_guid, topics, start, end, ros_distro='kinetic'):
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
                            "topics": topics,
                            "services": [],
                            "actions": [],
                            "isROS": True,
                            "ros_distro": ros_distro
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
                            {"default": str(end), "name": "END", "description": ""},
                            {"default": str(start), "name": "START", "description": ""}
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
    # pub_pkg = json.dumps(publisher_manifest)
    package_details = client.create_package(publisher_manifest)
    print('created publisher package...')
    return package_details

def deprovison_all_deps(pkg_ids):
    for pkg_id in pkg_ids:
        pkg = client.get_package(pkg_id)
        print('deprovisioning all deployments of this package {}'.format(pkg.packageName))
        deps = pkg.deployments(phases=[DeploymentPhaseConstants.SUCCEEDED, DeploymentPhaseConstants.PROVISIONING, DeploymentPhaseConstants.INPROGRESS])
        for dep in deps:
            dep.deprovision()
            print('deprovisioned {}'.format(dep.name))

def provision_pkg(native_network_guid, publisher_build_guid, topics, start, end, ros_distro='kinetic'):
    package_ids = []
    for i in range(start, end+1):
        print('creating package...')
        publisher_package_details = create_publisher_package(publisher_build_guid, topics, start, end, ros_distro)
        publisher_package_id = publisher_package_details['packageId']
        pkg = client.get_package(publisher_package_id)
        plan_id = pkg.plans[0].planId
        package_ids.append(publisher_package_id)
        cfg = pkg.get_provision_configuration(plan_id)
        net = client.get_native_network(native_network_guid)
        cfg.add_native_network(net)
        print('provisioning {} package'.format(pkg.packageName))
        dep = pkg.provision('publisher' + '-' + get_random_string(3), cfg)
        print('successfully started provisioning {}'.format(dep.name))
        dep_polling_till_phase_succeed(dep)
        print('provisioned {} package'.format(pkg.packageName))
    return package_ids



if __name__ == '__main__':
    client = Client(auth_token=AUTH_TOKEN, project=PROJECT_ID)
    if sys.argv[1] == '--provision':
        print('provision started...')
        start = int(sys.argv[2])
        end = int(sys.argv[3])
        count = end - start + 1
        # create publisher package
        topics = []
        for i in range(start, end + 1):
            topics.append({
                "name": "/chatter{}".format(i),
                "qos": "low",
                "compression": "",
                "scoped": False,
                "targeted": True
            })
        # provisioning publisher package
        provision_pkg(native_network_guid, publisher_build_guid, topics, start, end)

    elif sys.argv[1] == '--deprovision':
        print('deprovision started...')
        deprovison_all_deps(package_ids)

    elif sys.argv[1] == '--all':
        dep_num = int(sys.argv[2])  # num of deployment
        start_dep = 0  # start dep
        topic_num = 5  # number of topic each deployment
        ros_distro = ROSDistro.KINETIC # ros distro
        if len(sys.argv) >= 4:
            start_dep = int(sys.argv[3])
        if len(sys.argv) >= 5:
            topic_num = int(sys.argv[4])
        if len(sys.argv) >= 6:
            ros_distro = sys.argv[5]

        # creating build
        # publisher_build = create_build(ros_distro)
        # publisher_build_guid = publisher_build.guid
        #
        # # creating native network
        # native_network = create_native_network(ros_distro)
        # native_network_guid = native_network.guid

        for n in range(start_dep, dep_num):
            start = (n * topic_num) + 1
            end = (n * topic_num) + topic_num
            count = end - start + 1
            # create publisher package
            topics = []
            for i in range(start, end+1):
                topics.append({
                    "name": "/chatter{}".format(i),
                    "qos": "low",
                    "compression": "",
                    "scoped": False,
                    "targeted": False
                })
            # provisioning publisher package
            package_ids = provision_pkg(native_network_guid, publisher_build_guid, topics, start, end, ros_distro)

        print('publisher_build_guid ' + publisher_build_guid)
        print('native_network_guid ' + native_network_guid)
        print('package_ids {}'.format(package_ids))
        print('last topic number {}'.format((dep_num * topic_num)))
        print('last dep number {}'.format(dep_num))

