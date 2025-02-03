class RobotRequestData:
    def __init__(self, request_id=None, no=None, rail_pos=None, command=None, param1=None, param2=None, param3=None,param4=None,param5=None):
        """
        클래스 생성자

        :param request_id : 로봇 서비스 정보 아이디
        :param no : 로봇 서비스 정보 index
        :param command : 로봇 서비스에 사용 되는 명령어
        :param param1 : 추가 parameter 1
        :param param2 : 추가 parameter 2
        :param param3 : 추가 parameter 3
        :param param4 : 추가 parameter 4
        :param param5 : 추가 parameter 5

        """

        # 구분
        self.request_id = request_id
        self.no = no

        # 결과
        self.rail_pos = rail_pos
        self.command = command
        self.param1 = param1
        self.param2 = param2
        self.param3 = param3
        self.param4 = param4
        self.param5 = param5
