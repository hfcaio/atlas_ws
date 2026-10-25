#!/usr/bin/env python3
"""No que carrega a missao ARTEX 2000 m e a geofence no FCU.

**Este no NAO arma e NAO descola.** A descolagem da missao ARTEX e' manual: o
piloto descola, sobe ate' ~100 m sobre a pista e so' entao passa para AUTO. Por
isso nao se usa `run_mission()` da `mavros_library`, que faria
GUIDED -> arm -> AUTO -> MISSION_START.

O que este no faz:
  1. liga-se ao veiculo e espera `state.connected`;
  2. constroi a fence de inclusao e a missao a partir de `artex27.mission` /
     `artex27.fence`, com os parametros de `config/artex27.yaml`;
  3. valida as duas localmente (contagem, sequencia, contencao no circulo UAS);
  4. faz push da fence e da missao;
  5. confirma e imprime a missao item a item.

Uso:
    ros2 run artex27 artex_mission
    ros2 run artex27 artex_mission --ros-args -p manual_land:=true
    ros2 launch artex27 artex_mission.launch.py approach_brg:=137.2
"""

import rclpy
from rclpy.node import Node

from mavros_library.sequences import init_all, new_mission, set_geofence, wait_for_connection

from artex27 import fence as fence_mod
from artex27 import geo
from artex27 import mission as mission_mod
from artex27 import profile


class ArtexMissionNode(Node):
    """Carrega missao + geofence do ARTEX no FCU, sem armar."""

    def __init__(self):
        super().__init__('artex_mission')

        self.declare_parameter('vehicle', 'mavros')
        self.declare_parameter('manual_land', False)
        # -1.0 = usar o rumo geometrico centro -> pista (137,2 deg).
        self.declare_parameter('approach_brg', -1.0)
        self.declare_parameter('loiter_radius_m', profile.LOITER_RADIUS_M)
        self.declare_parameter('climb_steps_m',
                               [float(a) for a in profile.CLIMB_STEPS_M])
        self.declare_parameter('descent_steps_m',
                               [float(a) for a in profile.DESCENT_STEPS_M])
        self.declare_parameter('home_amsl_m', mission_mod.HOME_AMSL_PLACEHOLDER)
        self.declare_parameter('push_fence', True)
        self.declare_parameter('fence_inset_m', fence_mod.FENCE_INSET_M)
        self.declare_parameter('fence_vertices', fence_mod.FENCE_VERTICES)
        self.declare_parameter('fence_circle', False)
        self.declare_parameter('abort_on_validation_error', True)
        self.declare_parameter('design_wind_ms', profile.DESIGN_WIND_MS)
        # -1.0 = elevacao do terreno DESCONHECIDA (pendencia). Nao inventar.
        self.declare_parameter('terrain_elev_amsl_m', -1.0)

        self.vehicle = self.get_parameter('vehicle').value
        init_all(self, [self.vehicle])

    def _p(self, name):
        return self.get_parameter(name).value

    def run(self):
        """Sequencia completa. Devolve 0 se tudo correu bem."""
        self.get_logger().warn(
            'ARTEX 2000 m: este no NAO arma e NAO descola. '
            'A descolagem e MANUAL -- o piloto sobe ate ~%.0f m sobre a pista '
            'e so entao passa para AUTO.' % geo.AUTO_ENTRY_ALT_M)

        manual_land = bool(self._p('manual_land'))

        mission = mission_mod.build_mission(
            manual_land=manual_land,
            approach_brg=float(self._p('approach_brg')),
            climb_steps_m=list(self._p('climb_steps_m')),
            descent_steps_m=list(self._p('descent_steps_m')),
            loiter_radius_m=float(self._p('loiter_radius_m')),
            home_amsl_m=float(self._p('home_amsl_m')))

        fence = fence_mod.build_fence(
            inset_m=float(self._p('fence_inset_m')),
            vertices=int(self._p('fence_vertices')),
            circle=bool(self._p('fence_circle')))

        errors = mission_mod.validate_mission(mission, manual_land=manual_land)
        errors += fence_mod.validate_fence(fence, mission)
        if errors:
            for e in errors:
                self.get_logger().error('VALIDACAO: ' + e)
            if self._p('abort_on_validation_error'):
                self.get_logger().error('A abortar antes de carregar nada no FCU.')
                return 1
        else:
            self.get_logger().info(
                'Missao valida: %d itens, fence de %d pontos, tudo dentro do '
                'circulo UAS.' % (len(mission), len(fence)))

        for line in mission_mod.describe(mission):
            self.get_logger().info(line)

        if not wait_for_connection(self, [self.vehicle]):
            self.get_logger().error('Sem ligacao ao veiculo.')
            return 1

        if self._p('push_fence'):
            self.get_logger().info('A carregar a geofence de inclusao...')
            set_geofence(self, self.vehicle, fence)
        else:
            self.get_logger().warn('push_fence=false: a geofence NAO foi carregada.')

        self.get_logger().info('A carregar a missao...')
        new_mission(self, self.vehicle, mission)

        self._report_profile()
        self.get_logger().warn(
            'Antes de armar: confirmar RTL_AUTOLAND=4 (com DO_RETURN_PATH_START '
            'na missao e RTL_AUTOLAND=0 o ARME FALHA) e correr '
            '`ros2 run artex27 artex_check`.')
        return 0

    def _report_profile(self):
        """Imprime o perfil vertical com os valores em uso."""
        self._report_airspace()
        for wind in (0.0, float(self._p('design_wind_ms'))):
            climb = profile.climb_profile(
                wind=wind, entry_alt_m=geo.AUTO_ENTRY_ALT_M,
                steps_m=list(self._p('climb_steps_m')),
                radius_m=float(self._p('loiter_radius_m')))
            self.get_logger().info(
                'Subida com %.0f m/s de vento: TECS_CLMB_MAX=%.2f m/s, '
                'gradiente %.1f %%, %.1f min, %.1f voltas, raio efetivo max %.0f m.'
                % (wind, climb['razao_ms'], climb['gradiente'] * 100,
                   climb['tempo_min'], climb['voltas'], climb['raio_efetivo_max_m']))
        descent = profile.descent_profile(
            wind=float(self._p('design_wind_ms')),
            steps_m=list(self._p('descent_steps_m')),
            radius_m=float(self._p('loiter_radius_m')))
        self.get_logger().info(
            'Descida: TECS_SINK_MAX=%.2f m/s, %.1f min, %.1f voltas.'
            % (descent['razao_ms'], descent['tempo_min'], descent['voltas']))

    def _report_airspace(self):
        """Verifica o topo da missao contra os tetos de espaco aereo."""
        elev = float(self._p('terrain_elev_amsl_m'))
        top = max(list(self._p('climb_steps_m')))
        for level, text in geo.airspace_check(top, None if elev < 0.0 else elev):
            {'ok': self.get_logger().info,
             'aviso': self.get_logger().warn,
             'erro': self.get_logger().error}[level](text)


def main(args=None):
    """Ponto de entrada do `ros2 run artex27 artex_mission`."""
    rclpy.init(args=args)
    node = ArtexMissionNode()
    code = 1
    try:
        code = node.run()
    except KeyboardInterrupt:
        node.get_logger().warn('Interrompido pelo utilizador.')
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return code


if __name__ == '__main__':
    raise SystemExit(main())
