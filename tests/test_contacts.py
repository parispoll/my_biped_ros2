"""Prevent opposite-foot or non-ground collisions from being called ground support."""

from types import SimpleNamespace as Item
import unittest
from my_robot_description.sensing.contacts import ground_contact_pairs


def message(first, second):
    return Item(
        contacts=[Item(collision1=Item(name=first), collision2=Item(name=second))]
    )


class GroundContactTests(unittest.TestCase):
    def setUp(self):
        self.foot = "my_biped::left_foot::left_foot_collision"
        self.ground = "ground_plane::ground::collision"

    def test_matches_foot_ground_in_either_order(self):
        for pair in ((self.foot, self.ground), (self.ground, self.foot)):
            self.assertEqual(ground_contact_pairs(message(*pair), "left"), [pair])

    def test_other_foot_is_not_left_foot_support(self):
        self.assertEqual(
            ground_contact_pairs(message(self.foot, self.ground), "right"), []
        )

    def test_wall_contact_is_not_ground_support(self):
        self.assertEqual(
            ground_contact_pairs(message(self.foot, "wall::box::collision"), "left"), []
        )

    def test_similar_collision_name_is_not_a_match(self):
        self.assertEqual(
            ground_contact_pairs(message(self.foot + "_extra", self.ground), "left"), []
        )

    def test_empty_message_contains_no_contact_evidence(self):
        self.assertEqual(ground_contact_pairs(Item(contacts=[]), "left"), [])

    def test_missing_names_cannot_be_classified(self):
        with self.assertRaises(ValueError):
            ground_contact_pairs(message("", self.ground), "left")
