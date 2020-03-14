#Created By: Logan Fillo
#Created On: 2019-03-18


from _pysimtest import case
from _pysimtest import constants as const
import pytest


"""
Test suite for _pysimtest.case
"""

class TestCase:

    def test_build_test_case_no_globs(self):
        sim_builder = case.SimTestBuilder()
        sim_builder.build_test_case("test")
        assert sim_builder._curr_test.name == "test"
        assert len(sim_builder._curr_objs) == 0

    def test_build_test_case_glob_obj(self):
        sim_builder = case.SimTestBuilder()
        sim_builder.add_object("test", const.POLE_MODEL,0,0,0,0,0,0,True)
        assert len(sim_builder._glob_objs) == 1
        sim_builder.build_test_case("test")
        assert len(sim_builder._curr_objs) == 1
        sim_builder.add_object("test", const.POLE_MODEL,0,0,0,0,0,0,False)
        # SHouldn't change
        assert len(sim_builder._glob_objs) == 1
        assert len(sim_builder._curr_objs) == 2

    def test_build_test_glob_test_data(self):
        sim_builder = case.SimTestBuilder()
        sim_builder.set_timeout(13, True)
        sim_builder.use_dynamics(True, True)
        sim_builder.use_ai(True, True)
        assert sim_builder._glob_timeout == 13
        assert sim_builder._glob_is_dynamic == True
        assert sim_builder._glob_use_ai == True
        sim_builder.build_test_case("test")
        assert sim_builder._curr_test.timeout == 13
        assert sim_builder._curr_test.is_dynamic == True
        assert sim_builder._curr_test.use_ai == True
        sim_builder.set_timeout(7, False)
        sim_builder.use_dynamics(False, False)
        sim_builder.use_ai(False, False)
        # SHouldn't change
        assert sim_builder._glob_timeout == 13
        assert sim_builder._glob_is_dynamic == True
        assert sim_builder._glob_use_ai == True

        assert sim_builder._curr_test.timeout == 7
        assert sim_builder._curr_test.is_dynamic == False
        assert sim_builder._curr_test.use_ai == False

    def test_build_test_glob_vhcl_data(self):
        sim_builder = case.SimTestBuilder()
        sim_builder.set_vehicle_position(1,2,3,True)
        sim_builder.set_vehicle_rotation(4,5,6,True)
        assert sim_builder._glob_vhcl_data[const.VEHICLE_X_POS] == 1
        assert sim_builder._glob_vhcl_data[const.VEHICLE_Y_POS] == 2
        assert sim_builder._glob_vhcl_data[const.VEHICLE_Z_POS] == 3
        assert sim_builder._glob_vhcl_data[const.VEHICLE_R_ROT] == 4
        assert sim_builder._glob_vhcl_data[const.VEHICLE_P_ROT] == 5
        assert sim_builder._glob_vhcl_data[const.VEHICLE_Y_ROT] == 6
        sim_builder.build_test_case("test")
        assert sim_builder._curr_vhcl.data[const.VEHICLE_X_POS] == 1
        assert sim_builder._curr_vhcl.data[const.VEHICLE_Y_POS] == 2
        assert sim_builder._curr_vhcl.data[const.VEHICLE_Z_POS] == 3
        assert sim_builder._curr_vhcl.data[const.VEHICLE_R_ROT] == 4
        assert sim_builder._curr_vhcl.data[const.VEHICLE_P_ROT] == 5
        assert sim_builder._curr_vhcl.data[const.VEHICLE_Y_ROT] == 6
        sim_builder.set_vehicle_position(0,0,0,False)
        sim_builder.set_vehicle_rotation(0,0,0,False)
        # SHouldn't change
        assert sim_builder._glob_vhcl_data[const.VEHICLE_X_POS] == 1
        assert sim_builder._glob_vhcl_data[const.VEHICLE_Y_POS] == 2
        assert sim_builder._glob_vhcl_data[const.VEHICLE_Z_POS] == 3
        assert sim_builder._glob_vhcl_data[const.VEHICLE_R_ROT] == 4
        assert sim_builder._glob_vhcl_data[const.VEHICLE_P_ROT] == 5
        assert sim_builder._glob_vhcl_data[const.VEHICLE_Y_ROT] == 6

        assert sim_builder._curr_vhcl.data[const.VEHICLE_X_POS] == 0
        assert sim_builder._curr_vhcl.data[const.VEHICLE_Y_POS] == 0
        assert sim_builder._curr_vhcl.data[const.VEHICLE_Z_POS] == 0
        assert sim_builder._curr_vhcl.data[const.VEHICLE_R_ROT] == 0
        assert sim_builder._curr_vhcl.data[const.VEHICLE_P_ROT] == 0
        assert sim_builder._curr_vhcl.data[const.VEHICLE_Y_ROT] == 0

    def test_set_vehicle_position_invalid_y(self):
        x,y,z = 0, 0, -5
        try:
            sim_builder = case.SimTestBuilder()
            sim_builder.set_vehicle_position(x,y,z, True)
            pytest.fail()
        except SystemExit:
            pass
        try:
            sim_builder = case.SimTestBuilder()
            sim_builder.set_vehicle_position(x,y,z, False)
            pytest.fail()
        except SystemExit:
            pass

    def test_build_test_glob_scene_data(self):
        sim_builder = case.SimTestBuilder()
        sim_builder.set_wave_scale(6, True)
        assert sim_builder._glob_scene_data[const.SCENE_WAVE_SCALE] == 1e-06
        sim_builder.build_test_case("test")
        assert sim_builder._curr_scene.data[const.SCENE_WAVE_SCALE] == 1e-06
        sim_builder.set_wave_scale(6.5, False)
        # SHouldn't change
        assert sim_builder._glob_scene_data[const.SCENE_WAVE_SCALE] == 1e-06
        assert sim_builder._curr_scene.data[const.SCENE_WAVE_SCALE] == 1*10**-6.5

    def test_get_result(self):
        sim_builder = case.SimTestBuilder()
        sim_builder.build_test_case("test")
        test = sim_builder.get_result()
        assert test.name == "test"
