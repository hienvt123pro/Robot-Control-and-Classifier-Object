import os
import psycopg2
import psycopg2.extras
import pandas as pd


class PostgresDatabase:
    def __init__(self):
        self.DB_HOST = "localhost"
        self.DB_NAME = "postgres"
        self.DB_USER = "postgres"
        self.DB_PASS = "hienvt123"


class RobotDatabase(PostgresDatabase):
    """
    Connect to postgresSql, purpose to save teaching pont data for drop-place of robot end-effector
    """

    def __init__(self):
        super().__init__()

    def save_into_database(self, p, size, x, y, z):
        cnt = psycopg2.connect(dbname=self.DB_NAME, user=self.DB_USER, password=self.DB_PASS, host=self.DB_HOST)
        cursor = cnt.cursor(cursor_factory=psycopg2.extras.DictCursor)
        sql = "INSERT INTO robot_pendant (point, size_type, x_robot, y_robot, z_robot) " \
              "VALUES (%s, %s, %s, %s, %s)"
        sql_where = (p, size, x, y, z)
        cursor.execute(sql, sql_where)
        cnt.commit()
        cursor.close()
        cnt.close()

    def update_into_database(self, p, size, x, y, z):
        cnt = psycopg2.connect(dbname=self.DB_NAME, user=self.DB_USER, password=self.DB_PASS, host=self.DB_HOST)
        cursor = cnt.cursor(cursor_factory=psycopg2.extras.DictCursor)
        sql = "UPDATE robot_pendant SET size_type=%s, x_robot=%s, y_robot=%s, z_robot=%s where point=%s"
        sql_where = (size, x, y, z, p)
        cursor.execute(sql, sql_where)
        cnt.commit()
        cursor.close()
        cnt.close()

    def get_table_comment(self):
        cnt = psycopg2.connect(dbname=self.DB_NAME, user=self.DB_USER, password=self.DB_PASS, host=self.DB_HOST)
        cursor = cnt.cursor(cursor_factory=psycopg2.extras.DictCursor)
        sql = "select description from pg_catalog.pg_description join pg_catalog.pg_class on pg_description.objoid = pg_class.oid " \
              "where relname = 'robot_pendant'"
        cursor.execute(sql)
        cmt = cursor.fetchone()
        cursor.close()
        cnt.close()
        return cmt[0]

    def read_from_database(self):
        cnt = psycopg2.connect(dbname=self.DB_NAME, user=self.DB_USER, password=self.DB_PASS, host=self.DB_HOST)
        cursor = cnt.cursor(cursor_factory=psycopg2.extras.DictCursor)
        sql = "SELECT * FROM robot_pendant"
        cursor.execute(sql)
        content = cursor.fetchall()
        cursor.close()
        cnt.close()
        return content

    def delete_on_database(self):
        cnt = psycopg2.connect(dbname=self.DB_NAME, user=self.DB_USER, password=self.DB_PASS, host=self.DB_HOST)
        cursor = cnt.cursor(cursor_factory=psycopg2.extras.DictCursor)
        sql = "truncate robot_pendant"
        cursor.execute(sql)
        cnt.commit()
        cursor.close()
        cnt.close()


robot_database = RobotDatabase()


class ProductDatabase(PostgresDatabase):
    """
        Connect to postgresSql, purpose to save data of detected product
    """

    def __init__(self):
        super().__init__()
        self.dirs = ["error_product", "export"]

    def check_dir(self):
        for d in self.dirs:
            if not os.path.isdir(d):
                os.makedirs(d)

    def save_into_database(self, datetime, name, product, size, color, note):
        cnt = psycopg2.connect(dbname=self.DB_NAME, user=self.DB_USER, password=self.DB_PASS, host=self.DB_HOST)
        cursor = cnt.cursor(cursor_factory=psycopg2.extras.DictCursor)
        sql = "INSERT INTO product_data (date_check, name_product, product_type, size_type, color_type, note) " \
              "VALUES (%s, %s, %s, %s, %s, %s)"
        sql_where = (datetime, name, product, size, color, note)
        cursor.execute(sql, sql_where)
        cnt.commit()
        cursor.close()
        cnt.close()

    def read_from_database(self):
        cnt = psycopg2.connect(dbname=self.DB_NAME, user=self.DB_USER, password=self.DB_PASS, host=self.DB_HOST)
        cursor = cnt.cursor(cursor_factory=psycopg2.extras.DictCursor)
        sql = "SELECT * FROM product_data"
        cursor.execute(sql)
        content = cursor.fetchall()
        cursor.close()
        cnt.close()
        return content

    def export_excel(self):
        data = self.read_from_database()
        if data:
            # assume data is stored in the variable 'data'
            df = pd.DataFrame(data, columns=["Date Time", "Name of product", "Product Type",
                                             "Size Type", "Color Type", "Note"])
            # save the dataframe to an Excel file
            df.to_excel('export/product_data.xlsx', index=False)
            return True
        return False


product_database = ProductDatabase()
