"""Testes da geometria da missao ARTEX, sem ROS a correr.

Verificam o que o utilizador tem de poder confiar sem ligar nada: a missao tem
a estrutura certa, cabe toda dentro do circulo UAS e respeita os 10 % de
gradiente. Nao arrancam nenhum no nem falam com o FCU.
"""

import pytest

from artex27 import fence as fence_mod
from artex27 import geo
from artex27 import mission as mission_mod
from artex27 import profile


def test_missao_automatica_tem_21_itens():
    """A variante de pouso automatico tem exatamente 21 itens."""
    items = mission_mod.build_mission()
    assert len(items) == 21


def test_sequencia_de_comandos():
    """A sequencia de comandos bate certo com a esperada."""
    items = mission_mod.build_mission()
    assert [it.command for it in items] == mission_mod.expected_sequence()


def test_retorno_antes_da_aterragem():
    """DO_RETURN_PATH_START (188) tem de vir antes do DO_LAND_START (189)."""
    cmds = [it.command for it in mission_mod.build_mission()]
    assert cmds.index(mission_mod.CMD_DO_RETURN_PATH_START) < \
        cmds.index(mission_mod.CMD_DO_LAND_START)
    assert cmds[-1] == mission_mod.CMD_LAND


@pytest.mark.parametrize('manual_land', [False, True])
def test_missao_valida(manual_land):
    """A validacao nao encontra problemas em nenhuma das duas variantes."""
    items = mission_mod.build_mission(manual_land=manual_land)
    assert mission_mod.validate_mission(items, manual_land=manual_land) == []


@pytest.mark.parametrize('manual_land', [False, True])
def test_tudo_dentro_do_circulo_uas(manual_land):
    """Todos os pontos posicionados caem dentro do circulo de 997,25 m."""
    for item in mission_mod.build_mission(manual_land=manual_land):
        if item.x_lat == 0.0 and item.y_long == 0.0:
            continue
        assert geo.uas_margin_m((item.x_lat, item.y_long)) >= 0.0


def test_fence_cobre_a_missao():
    """A fence de inclusao contem a missao toda."""
    assert fence_mod.validate_fence(
        fence_mod.build_fence(), mission_mod.build_mission()) == []


def test_gradiente_de_subida_abaixo_de_10_por_cento():
    """Com o vento de projeto, o gradiente fica nos 8 %."""
    p = profile.climb_profile()
    assert p['gradiente'] <= profile.GRADIENT_LIMIT
    assert p['razao_ms'] == pytest.approx(0.8, abs=1e-6)


def test_raio_efetivo_cabe_na_area():
    """O raio de loiter a 2000 m continua com folga ate' a borda da area."""
    raio = profile.loiter_radius_eff(profile.LOITER_RADIUS_M, 2000.0)
    assert raio < geo.UAS_RADIUS_M / 2.0
