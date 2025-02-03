import os
import sys
import traceback

from gui_launcher.SQLiteDataBase import SQLiteDatabase
from gui_launcher.RobotRequestData import RobotRequestData
from gui_launcher.DBConstants import DBFieldName

class RobotRequestObject:
    T_ROBOT_SERVICE_REQ = 'T_ROBOT_SERVICE_REQ'  # 로봇 서비스 요청 객체
    T_COMPONENT_INFO = 'T_COMPONENT_INFO'  # 장치 정보 테이블
    T_ROBOT_INFO = 'T_ROBOT_INFO'  # 로봇 정보 테이블
    def __init__(self, db_file_name=None):
        """
        클래스 생성자
        :param db_file_name : 데이터베이스 파일 명
        """
        if db_file_name:
            self.db_file_name = db_file_name
        else:
            # 실행되고 있는 프로젝트 root 조회
            project_root = os.path.abspath(os.path.join(sys.path[0], "../../../.."))

            # database file
            file_name = f"{project_root}/{SQLiteDatabase.BBS_DATABASE_NAME}"

            # print(f"bbs db file path: {file_name}")
            self.db_file_name = file_name

    def get_robot_request(self, request_id, no=0):
        """
        로봇 서비스 요청 정보 loading

        :param request_id : 로봇 서비스 요청 아이디
        :param no  : 로봇 서비스 요청 인덱스
        :return : 서비스 요청 데이터 응답
        """

        try:
            db = SQLiteDatabase()
            # 서비스 정보 조회 조건
            condition = dict()
            condition[DBFieldName.REQUEST_ID] = request_id
            condition[DBFieldName.NO] = no

            # print(condition)

            # 서비스 요청 객체 조회
            data_row = db.select_data(RobotRequestObject.T_ROBOT_SERVICE_REQ, conditions=condition)

            # print(data_row[DBFieldName.FIRST_RECORD])

            request_data = RobotRequestData(**data_row[DBFieldName.FIRST_RECORD])

            # print(request_data.command)

            return request_data
        
        except:
            print(traceback.format_exc())

    def get_syrup_request(self, request_id, param1, param2, param3 ):
        """
        로봇 서비스 요청 정보 loading

        :param request_id : 로봇 서비스 요청 아이디
        :param no  : 로봇 서비스 요청 인덱스
        :return : 서비스 요청 데이터 응답
        """

        try:
            db = SQLiteDatabase(self.db_file_name)

            # 서비스 정보 조회 조건
            condition = dict()
            condition[DBFieldName.REQUEST_ID] = request_id
            condition[DBFieldName.PARAM_1] = param1
            condition[DBFieldName.PARAM_2] = param2
            condition[DBFieldName.PARAM_3] = param3

            # print(condition)

            # 서비스 요청 객체 조회
            data_row = db.select_data(RobotRequestObject.T_ROBOT_SERVICE_REQ, conditions=condition)

            # print(data_row[DBFieldName.FIRST_RECORD])

            request_data = RobotRequestData(**data_row[DBFieldName.FIRST_RECORD])

            # print(request_data.command)

            return request_data

        except:
            print(traceback.format_exc())

