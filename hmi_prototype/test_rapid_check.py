"""The RAPID checker must CATCH things, not merely pass the repo.

A linter that reports nothing is indistinguishable from a linter that does nothing, so each
test here feeds it a defect it exists to find -- starting with the one that actually took the
virtual controller down on 2026-09-20.
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tools"))

import rapid_check  # noqa: E402


GOOD = """MODULE TG_Example(SYSMODULE)
    LOCAL CONST num nThing:=1;

    PROC DoThing()
        VAR num i;
        ! A one-line IF needs no ENDIF.
        IF nThing=1 GOTO done;
        IF nThing=2 THEN
            TPWrite "two";
        ENDIF
        FOR i FROM 1 TO 3 DO
            TPWrite "loop";
        ENDFOR
        WHILE nThing>5 DO
            nThing:=nThing-1;
        ENDWHILE
        done:
    ENDPROC

    LOCAL FUNC num Half(num v)
        RETURN v/2;
    ENDFUNC
ENDMODULE
"""


class TestCleanSourcePasses(unittest.TestCase):
    def test_a_well_formed_module_is_clean(self):
        self.assertEqual(rapid_check.check_text(GOOD, "good.sys"), [])

    def test_one_line_if_is_not_treated_as_a_block(self):
        """Every guard clause in TG_Comms is `IF x=1 GOTO y;`. Counting those as blocks
        would report an imbalance on a correct file -- the fastest way to get a linter
        switched off."""
        src = "MODULE M\n    PROC P()\n        IF a=1 GOTO z;\n        z:\n    ENDPROC\nENDMODULE\n"
        self.assertEqual(rapid_check.check_text(src, "m.sys"), [])


class TestTheOutageDefect(unittest.TestCase):
    """2026-09-20: `⚠` in a comment made TG_Comms.sys fail to load, which erased the SysMod
    and put T_ROB1 into system failure state."""

    def test_non_ascii_in_a_comment_is_caught(self):
        src = GOOD.replace("! A one-line IF", "! \u26a0 A one-line IF")
        findings = rapid_check.check_text(src, "bad.sys")
        self.assertTrue(findings)
        self.assertIn("non-ASCII", findings[0])
        self.assertIn("U+26A0", findings[0])

    def test_non_ascii_anywhere_is_caught(self):
        for char in ("\u2014", "\u2192", "\u00b1", "\u201c"):
            src = GOOD.replace('TPWrite "two";', f'TPWrite "{char}";')
            findings = rapid_check.check_text(src, "bad.sys")
            self.assertTrue(findings, f"{char!r} not reported")


class TestBlockBalance(unittest.TestCase):
    def test_missing_endproc(self):
        src = GOOD.replace("    ENDPROC\n", "", 1)
        findings = rapid_check.check_text(src, "bad.sys")
        self.assertTrue(any("never closed" in f or "closes" in f for f in findings), findings)

    def test_missing_endif(self):
        src = GOOD.replace("        ENDIF\n", "", 1)
        findings = rapid_check.check_text(src, "bad.sys")
        self.assertTrue(findings)

    def test_stray_endwhile(self):
        src = GOOD.replace("        done:\n", "        ENDWHILE\n        done:\n")
        findings = rapid_check.check_text(src, "bad.sys")
        self.assertTrue(any("ENDWHILE" in f for f in findings), findings)

    def test_keywords_inside_comments_and_strings_are_ignored(self):
        """Otherwise a comment explaining the ENDIF below shifts the balance."""
        src = GOOD.replace(
            '            TPWrite "two";',
            '            ! the ENDIF below closes this\n            TPWrite "ENDPROC";',
        )
        self.assertEqual(rapid_check.check_text(src, "m.sys"), [])


class TestNamespaceCollision(unittest.TestCase):
    def test_module_name_equal_to_routine_name(self):
        """Finding F-1: RAPID module names and global routine names share one namespace."""
        src = "MODULE TD05Test\n    PROC TD05Test()\n    ENDPROC\nENDMODULE\n"
        findings = rapid_check.check_text(src, "TD05Test.mod")
        self.assertTrue(any("F-1" in f for f in findings), findings)

    def test_the_mod_suffix_convention_is_accepted(self):
        src = "MODULE TD05Test_Mod\n    PROC TD05Test()\n    ENDPROC\nENDMODULE\n"
        self.assertEqual(rapid_check.check_text(src, "TD05Test.mod"), [])


class TestStrings(unittest.TestCase):
    def test_unterminated_string(self):
        src = GOOD.replace('TPWrite "two";', 'TPWrite "two;')
        findings = rapid_check.check_text(src, "bad.sys")
        self.assertTrue(any("unterminated string" in f for f in findings), findings)

    def test_an_exclamation_inside_a_string_is_not_a_comment(self):
        src = GOOD.replace('TPWrite "two";', 'TPWrite "hi! there";')
        self.assertEqual(rapid_check.check_text(src, "m.sys"), [])


class TestTheRepoIsClean(unittest.TestCase):
    def test_every_shipped_rapid_file_passes(self):
        root = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "abb"
        )
        checked = 0
        for dirpath, _dirs, files in os.walk(root):
            for filename in files:
                if filename.endswith((".sys", ".mod")):
                    path = os.path.join(dirpath, filename)
                    findings = rapid_check.check_file(path)
                    self.assertEqual(findings, [], f"{filename}: {findings}")
                    checked += 1
        self.assertGreater(checked, 5, "the walk stopped finding RAPID files")


if __name__ == "__main__":
    unittest.main(verbosity=2)
