"""Control pages, one per stage of bring-up."""
from .base import ControlPage
from .link_page import LinkPage
from .bench import BenchPage
from .gait import GaitPage
from .drive import DrivePage

PAGES = (LinkPage, BenchPage, GaitPage, DrivePage)

__all__ = ["ControlPage", "LinkPage", "BenchPage", "GaitPage", "DrivePage", "PAGES"]
