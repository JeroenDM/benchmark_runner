"""
Functions and decorators to enable logging to a mongodb database.
"""
# import git
# import datetime
import functools


def log_motion_command(func):
    """ Decorator to log movel, movep and movej commands to database. """

    @functools.wraps(func)  # make debugging easier
    def wrapper(self, start_config, pose_goal):
        self.logs.append("Logging motion command")
        return func(self, start_config, pose_goal)

    return wrapper


# def log_run_to_db(task, filepath):
#     # git the git commit hash
#     repo = git.Repo(search_parent_directories=True)

#     db_data = {
#         "author": "JeroenDM",
#         "date": datetime.datetime.utcnow(),
#         "filename": filepath,
#         "git": {
#             "branch": repo.active_branch.name,
#             "sha": repo.head.object.hexsha
#         }
#     }

#     DB.add_data(db_data)
