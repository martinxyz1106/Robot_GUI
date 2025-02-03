import os
import sqlite3
import sys
import traceback

from gui_launcher.DBConstants import DBFieldName


class SQLiteDatabase:
    """ SQLite 데이터베이스를 처리 클래스 """
    """
     예제 사용법:
     db = SQLiteDatabase()
     db.create_table("students", ["id INTEGER PRIMARY KEY", "name TEXT", "age INTEGER"])
     db.insert_data("students", (1, "John Doe", 25))
     db.insert_data("students", (2, "Jane Doe", 22))
     vo = db.select_data("students")
     vo = db.select_condition_data('students', columns=['id', 'name', 'age'], conditions={'id' : 1})
     print(vo)
    """

    # BBS 사용 데이터 베이스 이름
    BBS_DATABASE_NAME = 'bbs_data.db'

    def __init__(self, db_name= BBS_DATABASE_NAME):
        self.db_name = db_name
        self.conn = None
        self.cursor = None

    def connect(self):
        """
        데이터베이스에 연결하고 커서를 생성
        """
        try:
            self.conn = sqlite3.connect(self.db_name)
            self.cursor = self.conn.cursor()
            # print(f"Connected to SQLite database: {self.db_name}")
        except:
            print(traceback.format_exc())

    def create_table(self, table_name, columns):
        """
        테이블을 생성

        :param table_name : 테이블 이름
        :param columns    : 테이블 컬럼 들
        """
        try:
            self.connect()
            columns_str = ', '.join(columns)
            create_table_query = f"CREATE TABLE IF NOT EXISTS {table_name} ({columns_str})"
            self.cursor.execute(create_table_query)
            self.conn.commit()
            print(f"Table '{table_name}' created successfully.")
        except:
            print(traceback.format_exc())
        finally:
            self.close_connection()

    def insert_data(self, table_name, data):
        """
        테이블 데이터 추가

        :param table_name : 테이블 이름
        :param data       : 입력 데이터 들
        """
        try:
            self.connect()
            placeholders = ', '.join(['?' for _ in data])
            insert_data_query = f"INSERT INTO {table_name} VALUES ({placeholders})"
            self.cursor.execute(insert_data_query, data)
            self.conn.commit()
            print("Data inserted successfully.")

        except:
            print(traceback.format_exc())

        finally:
            self.close_connection()

    def upsert_data(self, table_name, data):
        """
        테이블 데이터 추가 또는 업데이트

        :param table_name: 테이블 이름
        :param data: 입력 데이터들
        """
        try:
            self.connect()
            placeholders = ', '.join(['?' for _ in data])
            columns = ', '.join(data.keys())
            values = tuple(data.values())

            # SQLite에서는 INSERT OR REPLACE 문을 사용하여 upsert 작업을 수행합니다.
            upsert_query = f"INSERT OR REPLACE INTO {table_name} ({columns}) VALUES ({placeholders})"
            self.cursor.execute(upsert_query, values)
            self.conn.commit()
            print("Data upserted successfully.")

        except:
            print(traceback.format_exc())

        finally:
            self.close_connection()

    def select_data(self, table_name, conditions=None):
        """
        테이블에서 데이터를 조회합니다

        :param table_name : 테이블 이름
        :param conditions  : 데이터 조건 dict {"column1": 42, "column2": "value"}

        :return: 데이터 조회 결과 list(dict)
        """
        try:
            self.connect()
            select_data_query = f" SELECT * FROM {table_name} "
            if conditions:
                where_conditions = [f"{column} = {value!r}" for column, value in conditions.items()]
                select_data_query += f" WHERE {' AND '.join(where_conditions)}"

            self.cursor.execute(select_data_query)

            result_set = self.cursor.fetchall()
            select_columns = [column[DBFieldName.FIRST_RECORD] for column in self.cursor.description]

            # Convert each row to a dictionary
            rows = [dict(zip(select_columns, row)) for row in result_set]
            return rows

        except:
            print(traceback.format_exc())
            return None

        finally:
            self.close_connection()

    def select_condition_data(self, table_name, columns=None, conditions=None):
        """
        테이블에서 데이터를 조회합니다

        :param table_name : 테이블 이름
        :param columns : 데이터를 원하는 컬럼
        :param conditions  : 데이터 조건 dict {"column1": 42, "column2": "value"}

        :return: 데이터 조회 결과 list(dict)
        """
        try:
            self.connect()
            select_data_query = f"SELECT {', '.join(columns)} FROM {table_name}"
            if conditions:
                where_conditions = [f"{column} = {value!r}" for column, value in conditions.items()]
                select_data_query += f" WHERE {' AND '.join(where_conditions)}"
            self.cursor.execute(select_data_query)
            result_set = self.cursor.fetchall()
            select_columns = [column[DBFieldName.FIRST_RECORD] for column in self.cursor.description]

            # Convert each row to a dictionary
            rows = [dict(zip(select_columns, row)) for row in result_set]
            return rows
        except:
            print(traceback.format_exc())
            return None

        finally:
            self.close_connection()

    def delete_condition_data(self, table_name, conditions):
        """
        테이블에서 특정 데이터를 삭제

        :param table_name : 테이블 이름
        :param conditions  : 데이터 조건 dict {"column1": 42, "column2": "value"}

        :return:
        """
        try:
            self.connect()
            delete_data_query = f"DELETE FROM {table_name}"
            where_conditions = [f"{column} = {value!r}" for column, value in conditions.items()]
            delete_data_query += f" WHERE {' AND '.join(where_conditions)}"
            self.cursor.execute(delete_data_query)
            self.conn.commit()
        except:
            print(traceback.format_exc())
        finally:
            self.close_connection()

    def close_connection(self):
        """
        데이터베이스 연결을 닫습니다.
        """
        if self.conn:
            self.conn.close()
            # print("Connection closed.")


if __name__ == '__main__':
    # project root dir
    project_root = os.path.abspath(os.path.join(sys.path[0], "../../../.."))

    # database file
    data_base = f"{project_root}/bbs_data.db"
    db = SQLiteDatabase(data_base)
    data = db.select_condition_data('students', columns=['id', 'name', 'age'], conditions={'id': 1})
    print(data)
