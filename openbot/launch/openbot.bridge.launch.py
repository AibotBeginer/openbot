# Copyright (c) 2024 OpenRobotic Beginner Authors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import subprocess

# 定义要并发执行的命令列表
commands = [
    ['cyber_launch', 'start', '/opt/openbot/share/openbot/bridge/launch/grpc_bridge.launch'],
]

def start_env():
    command = "source /opt/cyber/setup.zsh && env"
    proc = subprocess.Popen(command, shell=True, executable='/bin/zsh', stdout=subprocess.PIPE)
    for line in proc.stdout:
        (key, _, value) = line.decode('utf-8').partition("=")
        os.environ[key] = value.strip()

    proc.communicate()

# 存储所有的 Popen 对象
processes = []

def start_cmds():
    # 启动所有命令
    for command in commands:
        print(f"Starting command: {command}")
        # process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        process = subprocess.Popen(command, stdout=None, stderr=None)
        processes.append(process)

    # 等待所有进程完成
    for process in processes:
        stdout, stderr = process.communicate()
        print(f"Command finished with return code {process.returncode}")
        if stdout:
            print("Output:", stdout.decode())
        if stderr:
            print("Error:", stderr.decode())

def main():
    #start_env()
    start_cmds()

if __name__ == "__main__":
    main()
