from pathlib import Path

from dataclasses import fields

from typing import Optional

from pathlib import Path

import sqlite3

from ament_index_python.packages import get_package_share_directory

from ariac_db.structures import Trial, Run, OrderSubmission, Penalty

class DatabaseError(Exception):
    """Custom exception for database error"""
    def __init__(self, message):
        super().__init__(message)
        self.message = message

class DatabaseManager:
    def __init__(self, db_path: Path, create=False):
        self.db_path = db_path

        if create:
            self._create()
        
        with self._connect_to_db():
            pass
    
    def __enter__(self):
        return self
    
    def __exit__(self, exc_type, exc_value, traceback):
        pass

    def _create(self):
        # Read schema
        share = Path(get_package_share_directory('ariac_db'))
        schema = share.joinpath('schema', 'ariac.sql')
        
        if not schema.exists():
            raise DatabaseError(f'Unable to locate ariac db schema at {schema}')
        
        sql_script = schema.read_text()

        # Create database from schema
        try:
            with self._connect_to_db(create=True) as conn:
                conn.executescript(sql_script)

        except Exception as e:
            raise DatabaseError(f'Database creation failed: {e}')

    def clear_tables(self):
        try:
            with self._connect_to_db() as conn:
                cursor = conn.cursor()

                # Get all user-defined tables (ignore SQLite internal tables)
                cursor.execute("SELECT name FROM sqlite_master WHERE type='table' AND name NOT LIKE 'sqlite_%';")
                tables = cursor.fetchall()

                for (table_name,) in tables:
                    print(f"Clearing table: {table_name}")
                    cursor.execute(f"DELETE FROM {table_name};")  # Clears data
        
        except Exception as e:
            raise DatabaseError(f'Error clearing database tables: {e}')
        
    
    def get_trial_for_run(self, run: Run) -> Optional[Trial]:
        try:
            with self._connect_to_db() as conn:
                cursor = conn.cursor()

                query = f"SELECT {", ".join([f.name for f in fields(Trial)])} from Trial WHERE id={run.trial_id}"

                for row in cursor.execute(query):
                    return Trial(**row)

        except Exception as e:
            raise DatabaseError(f'Error reading from database: {e}')

        return None
    
    def get_all_trial_ids(self) -> list[str]:
        trial_ids = []
        try:
            with self._connect_to_db() as conn:
                cursor = conn.cursor()
                
                query = f"SELECT trial_id from Trial"

                for row in cursor.execute(query):
                    trial_ids.append(row["trial_id"])

        except Exception as e:
            raise DatabaseError(f'Error reading from database: {e}')
        
        return trial_ids

    def get_all_competitor_names(self) -> list[str]:
        names = []
        try:
            with self._connect_to_db() as conn:
                cursor = conn.cursor()
                
                query = f"SELECT name from Competitor"

                for row in cursor.execute(query):
                    names.append(row["name"])
                    
        except Exception as e:
            raise DatabaseError(f'Error reading from database: {e}')
        
        return names

    def get_competitor_name_for_run(self, run_id: int) -> str:
        try:
            with self._connect_to_db() as conn:
                cursor = conn.cursor()

                query = f"SELECT name FROM Competitor WHERE id=(SELECT competitor_id FROM Run where id={run_id})"

                for row in cursor.execute(query):
                    return row["name"]
        except Exception as e:
            raise DatabaseError(f"Error reading from database: {e}")
        
        return ""
    
    
    def get_trial_by_id(self, trial_id: str) -> Optional[Trial]:
        try:
            with self._connect_to_db() as conn:
                cursor = conn.cursor()

                query = f"SELECT {", ".join([f.name for f in fields(Trial)])} from Trial WHERE trial_id='{trial_id}'"

                for row in cursor.execute(query):
                    return Trial(**row)

        except Exception as e:
            raise DatabaseError(f'Error reading from database: {e}')

        return None
    
    
    def get_run(self, run_id: int) -> Optional[Run]:
        try:
            with self._connect_to_db() as conn:
                cursor = conn.cursor()

                query = f"SELECT {", ".join([f.name for f in fields(Run)])} FROM Run WHERE id={run_id}"

                for row in cursor.execute(query):
                    return Run(**row)

        except Exception as e:
            raise DatabaseError(f'Error reading from database: {e}')

        return None
    
    
    def run_complete(self, run_id: int) -> Optional[bool]:
        try:
            with self._connect_to_db() as conn:
                cursor = conn.cursor()

                query = f"SELECT {", ".join([f.name for f in fields(Run)])} FROM Run WHERE id={run_id}"

                for row in cursor.execute(query):
                    return row["completed"]

        except Exception as e:
            raise DatabaseError(f'Error reading from database: {e}')

        return None

    
    def get_orders_for_run(self, run_id: int) -> list[OrderSubmission]:
        try:
            with self._connect_to_db() as conn:
                cursor = conn.cursor()
                
                columns = ", ".join([f.name for f in fields(OrderSubmission)])
                query = f"SELECT {columns} FROM OrderSubmission WHERE run_id={run_id}"
                
                return [OrderSubmission(**row) for row in cursor.execute(query)]

        except Exception as e:
            raise DatabaseError(f'Error reading from database: {e}')
    
    
    def get_penalties_for_run(self, run_id: int) -> list[Penalty]:
        try:
            with self._connect_to_db() as conn:
                cursor = conn.cursor()
                
                columns = ", ".join([f.name for f in fields(Penalty)])
                query = f"SELECT {columns} FROM Penalty WHERE run_id={run_id}"

                return [Penalty(**row) for row in cursor.execute(query)]

        except Exception as e:
            raise DatabaseError(f'Error reading from database: {e}')
    
    
    def get_similar_run_ids(self, run: Run):
        try:
            with self._connect_to_db() as conn:
                cursor = conn.cursor()

                query = f"SELECT id FROM Run WHERE (trial_id=(SELECT id FROM Trial WHERE id={run.trial_id}) AND competitor_id={run.competitor_id})"

                return [row["id"] for row in cursor.execute(query)]

        except Exception as e:
            raise DatabaseError(f'Error reading from database: {e}')
        
    
    def get_run_ids_for_trial(self, trial_id: str, competitor_name: str) -> list[int]:
        try:
            with self._connect_to_db() as conn:
                cursor = conn.cursor()

                query = f"SELECT id FROM Run WHERE (trial_id=(SELECT id FROM Trial WHERE trial_id='{trial_id}') AND competitor_id=(SELECT id FROM Competitor where name='{competitor_name}'))"

                return [row["id"] for row in cursor.execute(query)]

        except Exception as e:
            raise DatabaseError(f'Error reading from database: {e}')

    
    def _connect_to_db(self, create=False) -> sqlite3.Connection:
        if not self.db_path.exists() and not create:
            raise DatabaseError(f'Database at {self.db_path} does not exist')
        
        try:
            conn = sqlite3.connect(self.db_path)
            conn.row_factory = sqlite3.Row
            return conn
        except Exception as e:
            raise DatabaseError(f'ERROR unable to connect to database: {e}')
        
