import os
import json
import shutil
import schedule
import time
import logging

class Score:
    def __init__(self, data: dict) -> None:
        self.data = data

    def __lt__(self, other):
        if self.data["Realtime"] == "unqualified":
            return False
        elif other.data["Realtime"] == "unqualified":
            return True
        if self.data["APE"]["rmse"] < 5.0 and other.data["APE"]["rmse"] < 5.0:  # A类排名
            if abs(self.data["APE"]["rmse"] - other.data["APE"]["rmse"]) > 0.0001:
                return self.data["APE"]["rmse"] < other.data["APE"]["rmse"]
            else:
                return (
                    self.data["ValidFramePercentage"]
                    < other.data["ValidFramePercentage"]
                )
        elif self.data["RPE"]["rmse"] > 5.0 and other.data["RPE"]["rmse"] > 5.0:  # B类排名
            if abs(self.data["RPE"]["rmse"] - other.data["RPE"]["rmse"]) > 0.0001:
                return self.data["RPE"]["rmse"] < other.data["RPE"]["rmse"]
            else:
                return (
                    self.data["ValidFramePercentage"]
                    < other.data["ValidFramePercentage"]
                )
        else:
            return self.data["APE"]["rmse"] < other.data["APE"]["rmse"]

    def add_rank(self, rank: int):
        self.data["rank"] = rank

class ResultUpdater():
    def __init__(self, logger:logging.Logger) -> None:
        self.logger = logger
    def get_best_score(self, scores: list) -> Score:
        assert len(scores) != 0
        bestScore = scores[0]
        for score in scores:
            if not isinstance(score, Score):
                return None
            if score < bestScore:
                bestScore = score
        return bestScore
    def get_json_data(self, root: str, file: str) -> dict:
        file_path = os.path.join(root, file)
        data = {}
        try:
            with open(file_path, "r", encoding="utf-8") as f:
                try:
                    data = json.load(f)
                except json.JSONDecodeError:
                    self.logger.error(f"[result updater] 无法解析文件内容: {file_path}")
        except FileNotFoundError:
            self.logger.error(f"[result updater] 文件 {file_path} 不存在")
        except IOError:
            self.logger.error(f"读取文件 {file_path} 发生错误")

        return data


    def Update(self, collect_folder:str, result_save_folder:str):
        destination_file = os.path.join(result_save_folder, "ready_publish_result.json")
        team_scores = {}  # 存储所有队伍的所有成绩
        best_scores = []  # 存储所有队伍的最好成绩
        for root, _, files in os.walk(collect_folder):
            for file in files:
                if file.endswith(".json"):
                    data = self.get_json_data(root, file)
                    team = data["team"]
                    if team not in team_scores:
                        team_scores[team] = []
                    team_scores[team].append(Score(data))
        if len(team_scores) == 0:
            self.logger.info("[result updater]没有找到json文件")
        for team, scores in team_scores.items():
            best_score = self.get_best_score(scores)
            best_scores.append(best_score)
        sorted_data = sorted(best_scores)
        try:
            with open(destination_file, "w", encoding="utf-8") as f:
                data_collect = []
                rank = int(1)
                for score in sorted_data:
                    assert isinstance(score, Score)
                    score.add_rank(rank)
                    rank = rank + 1
                    data_collect.append(score.data)
                json.dump(data_collect, f, indent=4, ensure_ascii=False)
                self.logger.info("[result updater]数据汇总写入成功")
        except IOError:
            self.logger.error("[result updater]写入文件发生错误")