import os
import time
import docker
import logging
import subprocess
import json
class DockerUpdater:
    def __init__(self, image_folder_path, history_image_path,seq:str) -> None:
        self.seq = seq
        self.logger = logging.getLogger('./logs/update_log')
        self.InitLogger()
        self.docker_cilent = docker.from_env()
        self.image_folder_path = image_folder_path
        if not os.path.exists(history_image_path):
            try:
                os.makedirs(history_image_path)
                self.logger.info(f"文件夹 {history_image_path} 创建成功")
            except OSError as e:
                self.logger.error(f"无法创建文件夹 {history_image_path}: {e}")
        self.hisoty_image_path = history_image_path
        self.StopAllDockerContainer()
        self.ClearImages()
    def StopAllDockerContainer(self):
        # 获取所有正在运行的容器
        containers = self.docker_cilent.containers.list()
        # 停止每个容器
        for container in containers:
            try:
                container.stop()
                self.logger.info(f"已停止容器: {container.id}")
            except Exception as e:
                self.logger.error(f"无法停止容器 {container.id}: {e}")
        self.logger.info("所有容器已停止")
    def ClearImages(self):
        # 获取所有的镜像
        images = self.docker_cilent.images.list()
        # 删除每个镜像
        for image in images:
            try:
                self.docker_cilent.images.remove(image.id)
                self.logger.info(f"已删除镜像: {image.id}")
            except Exception as e:
                self.logger.error(f"无法删除镜像 {image.id}: {e}")
        self.logger.info("所有镜像已删除")
    def GetImageName(self)->str:
        images = self.docker_cilent.images.list()
        assert(len(images) == 1)
        return images[0].id
    def ExcShell(self,team:str,school:str,image:str, no:str):
        shell_script_path = 'refree_docker_auto.sh'
        try:
            subprocess.run(['bash', shell_script_path, self.seq, team, image, no], check=True)
            self.logger.info("脚本执行成功")
        except subprocess.CalledProcessError as e:
            self.logger.error(f"脚本执行失败，返回码: {e.returncode}")
    def InitLogger(self):
        logging.basicConfig(level=logging.INFO,
                    format='%(asctime)s %(levelname)s %(message)s',
                    filename='./logs/update_log',
                    filemode='w')
        fh = logging.FileHandler('./logs/update_log')
        fh.setLevel(logging.DEBUG)
        ch = logging.StreamHandler()
        ch.setLevel(logging.DEBUG)
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        ch.setFormatter(formatter)
        fh.setFormatter(formatter)
        self.logger.addHandler(ch)
        self.logger.addHandler(fh)
    def GetSchoolAndTeam(self, image_name:str):
        parts = image_name.split(".")[0].split("_")
        return parts[0],parts[1]
    def UpdateResultJSON(self, team:str, school:str, no:str):
        outputjson = os.path.join(os.getcwd(),"result", team, f"seq{self.seq}",f"no{no}", f"{team}_result.json")
        with open(outputjson, 'r') as file:
            data = json.load(file)
        res = {"no":no,"team":team, "school":school}
        data.update(res)
        with open(outputjson, 'w') as file:
            json.dump(data, file, indent=4)
            self.logger.info(f"运行结束， 结果保存在: {outputjson}\n")
    def Update(self):
        files = os.listdir(self.image_folder_path)
        tar_files = [file for file in files if file.endswith('.tar')]
        if len(tar_files) == 0:
            return
        for tar_file in tar_files:
            tar_path = os.path.join(self.image_folder_path, tar_file)
            self.logger.info(f"正在处理: {tar_path}")
            with open(tar_path, 'rb') as image_file:
                self.logger.info(f"正在加载镜像: {tar_path}")
                response = self.docker_cilent.images.load(image_file.read())
                self.logger.info(f"加载镜像成功: {tar_path}. Info: {response}")
            os.rename(tar_path, os.path.join(self.hisoty_image_path, tar_file))
            school,team =self.GetSchoolAndTeam(tar_file)
            image = self.GetImageName()
            for no in range(1,6):
                self.logger.info(f"学校: {school}, 队伍: {team}, 镜像: {image} 正在运行第{no}次.")
                self.ExcShell(team, school,image, str(no))
                self.UpdateResultJSON(team,school, str(no))
            self.StopAllDockerContainer()
            self.ClearImages()
if __name__ == "__main__":
    image_folder_path = "ContestJZDW" 
    docker_updater = DockerUpdater(image_folder_path,"./ContestJZDW/history","3")
    while True:
        docker_updater.Update()
        time.sleep(1)