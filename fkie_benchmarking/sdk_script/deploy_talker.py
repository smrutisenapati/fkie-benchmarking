import sys
import random
import string
from rapyuta_io import Client, ROSDistro, SimulationOptions, BuildOptions, CatkinOption

AUTH_TOKEN = 'WxWkvMADRrnZzdXPi2Dy3a9m4410NT8sXcqEqYzU'
PROJECT_ID = 'project-zxfsnuxqpomamdcjgvznhccv'

client = Client(AUTH_TOKEN, PROJECT_ID)

def get_random_string(len):
    return ''.join(random.sample(string.lowercase, len))

def create_build():
    simulationOptions = SimulationOptions(False)
    buildOptions = BuildOptions(catkinOptions=[CatkinOption(rosPkgs='talker')])
    build = Build(buildName='{}-{}'.format('fkie_benchmarking', get_random_string(2)),
                  strategyType='Source',
                  repository='https://github.com/smrutisenapati/fkie-benchmarking',
                  architecture='amd64',
                  rosDistro='kinetic',
                  isRos=True,
                  contextDir='talk/talker',
                  simulationOptions=simulationOptions,
                  buildOptions=buildOptions)
    build = client.create_build(build)
    build.poll_build_till_ready()

def create_package():
    pass

def deploy_package():
    pass

if __name__ == '__main__':
    for arg in sys.argv[1:]:
        if arg == '--build':
            create_build()
        elif arg == '--create':
            create_package()
        elif arg == '--deploy':
            deploy_package()
        else:
            pass




