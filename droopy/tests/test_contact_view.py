import os
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
from dataclasses import replace
import numpy as np
from PySide6.QtWidgets import QApplication
from droopy.physics import Simulator,Config
from droopy.contact_view import ContactSchematic


def test_world_xy_projection_and_sum_drop_only_vertical_support(tmp_path):
    app=QApplication.instance() or QApplication([])
    sim=Simulator();sim.position[2]-=.001;sim.velocity[:]=[.2,-.1,-.03];sim.step()
    view=ContactSchematic();view.set_contacts(sim.last_diagnostic.contacts,'Captured step: test')
    np.testing.assert_allclose(view.net_force_xy,sim.last_diagnostic.contact_force_n[:2])
    for c,f in zip(view.contacts,view.projected_forces):
        assert f==c.force_world_n[:2]
    raised=[replace(c,force_world_n=(c.force_world_n[0],c.force_world_n[1],c.force_world_n[2]+100)) for c in view.contacts]
    before=view.projected_forces;view.set_contacts(raised)
    assert view.projected_forces==before
    opposing=[replace(view.contacts[0],force_world_n=(2.,-3.,10.)),replace(view.contacts[1],force_world_n=(-2.,3.,20.))]
    view.set_contacts(opposing)
    assert view.net_force_xy==(0.,0.)
    assert view.projected_forces[0]!=(0.,0.)
    view.set_contacts(raised)
    assert 'yellow means loaded' in view.accessibleDescription()
    view.resize(400,470);view.show();app.processEvents()
    assert view.grab().save(str(tmp_path/'ghost.png'))
    view.set_contacts(())
    assert view.net_force_xy==(0,0)
    view.close()


def test_loaded_frictionless_feet_have_zero_ghost_force():
    app=QApplication.instance() or QApplication([])
    sim=Simulator(Config(friction=0));sim.position[2]-=.001;sim.velocity[0]=1;sim.step()
    view=ContactSchematic();view.set_contacts(sim.last_diagnostic.contacts)
    assert any(c.normal_n>0 for c in view.contacts)
    assert view.net_force_xy==(0,0)
    assert all(f==(0,0) for f in view.projected_forces)
    view.close()
