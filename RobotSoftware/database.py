import psycopg2
import psycopg2.extras


class MyDatabase:
    """
    Connect to postgresSql, purpose to save teaching pont data for drop-place of robot end-effector
    """
    def __init__(self):
        self.DB_HOST = "localhost"
        self.DB_NAME = "postgres"
        self.DB_USER = "postgres"
        self.DB_PASS = "hienvt123"

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


my_database = MyDatabase()
