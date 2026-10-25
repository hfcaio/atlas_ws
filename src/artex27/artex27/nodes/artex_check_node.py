#!/usr/bin/env python3
"""Pre-voo automatizado: le o FCU e reporta divergencias face ao esperado.

Confere tres coisas, por esta ordem:
  1. **missao** -- numero de itens, sequencia de comandos, contencao no circulo
     UAS e a ordem DO_RETURN_PATH_START -> DO_LAND_START -> LAND;
  2. **geofence** -- existe, e' de inclusao e cobre a missao toda;
  3. **parametros** -- compara com a tabela de `artex27.params`.

Nao escreve nada no FCU. Sai com codigo 0 se estiver tudo conforme.

Uso:
    ros2 run artex27 artex_check
    ros2 run artex27 artex_check --ros-args -p manual_land:=true
"""

import time

import rclpy
from rclpy.node import Node

from mavros_library.config import SERVICES
from mavros_library.sequences import init_all, wait_for_connection

from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters

from mavros_msgs.srv import ParamPull

from artex27 import fence as fence_mod
from artex27 import geo
from artex27 import mission as mission_mod
from artex27 import params as params_mod


class ArtexCheckNode(Node):
    """Le missao, fence e parametros do FCU e compara com o esperado."""

    def __init__(self):
        super().__init__('artex_check')

        self.declare_parameter('vehicle', 'mavros')
        self.declare_parameter('manual_land', False)
        self.declare_parameter('check_params', True)
        self.declare_parameter('timeout_s', 20.0)

        self.vehicle = self.get_parameter('vehicle').value
        init_all(self, [self.vehicle])

        # O MAVROS 2 expoe os parametros do FCU pela API de parametros do ROS 2
        # (`param/get_parameters`), nao pelo servico `ParamGet` do mavros_msgs,
        # que esta' marcado como DEPRECATED. `param/pull` forca a sincronizacao
        # com o FCU antes da leitura.
        self.param_get_client = self.create_client(
            GetParameters, SERVICES[self.vehicle]['Param']['Get Parameters'])
        self.param_pull_client = self.create_client(
            ParamPull, SERVICES[self.vehicle]['Param']['Pull'])

        self.findings = []

    def _p(self, name):
        return self.get_parameter(name).value

    def _add(self, level, text):
        self.findings.append((level, text))
        logger = {'ok': self.get_logger().info,
                  'aviso': self.get_logger().warn,
                  'erro': self.get_logger().error}[level]
        logger(f'[{level.upper()}] {text}')

    # -- leitura ------------------------------------------------------------
    def _wait_for(self, getter, timeout_s):
        """Gira o executor ate' `getter()` devolver algo nao vazio."""
        deadline = time.time() + timeout_s
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            value = getter()
            if value:
                return value
        return None

    def _pull_params(self, timeout_s=30.0):
        """Forca o MAVROS a puxar a tabela de parametros do FCU."""
        if not self.param_pull_client.wait_for_service(timeout_sec=5.0):
            return False
        req = ParamPull.Request()
        req.force_pull = False
        future = self.param_pull_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
        result = future.result()
        return bool(result and result.success)

    def _fcu_params(self, names, timeout_s=15.0):
        """Le varios parametros do FCU de uma vez. Valores em falta vem a `None`."""
        out = {n: None for n in names}
        if not self.param_get_client.wait_for_service(timeout_sec=5.0):
            return out
        req = GetParameters.Request()
        req.names = list(names)
        future = self.param_get_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
        result = future.result()
        if not result:
            return out
        for name, value in zip(names, result.values):
            if value.type == ParameterType.PARAMETER_DOUBLE:
                out[name] = float(value.double_value)
            elif value.type == ParameterType.PARAMETER_INTEGER:
                out[name] = float(value.integer_value)
            elif value.type == ParameterType.PARAMETER_BOOL:
                out[name] = float(value.bool_value)
        return out

    # -- verificacoes -------------------------------------------------------
    def check_mission(self):
        """Compara a missao que esta' no FCU com a que este pacote gera."""
        vehicle_node = getattr(self, self.vehicle)
        waypoints = self._wait_for(
            lambda: list(vehicle_node.mission_waypoints.waypoints),
            float(self._p('timeout_s')))
        if not waypoints:
            self._add('erro', 'Nao chegou nenhuma missao do FCU '
                              '(topico mission/waypoints vazio).')
            return

        manual_land = bool(self._p('manual_land'))
        expected = mission_mod.expected_sequence(manual_land)
        self._add('ok' if len(waypoints) == len(expected) else 'erro',
                  f'Missao no FCU: {len(waypoints)} itens '
                  f'(esperados {len(expected)}).')

        # O ArduPilot reescreve o item 0 (home) com a posicao real de arme, por
        # isso a comparacao de comandos comeca no indice 1.
        for i, (got, want) in enumerate(zip(waypoints[1:], expected[1:]), start=1):
            if got.command != want:
                self._add('erro', f'Item {i}: comando {got.command} no FCU, '
                                  f'esperado {want}.')

        outside = []
        for i, wp in enumerate(waypoints):
            if wp.x_lat == 0.0 and wp.y_long == 0.0:
                continue
            margin = geo.uas_margin_m((wp.x_lat, wp.y_long))
            if margin < 0.0:
                outside.append((i, -margin))
        if outside:
            for i, d in outside:
                self._add('erro', f'Item {i} esta {d:.0f} m FORA do circulo UAS.')
        else:
            self._add('ok', 'Todos os itens posicionados estao dentro do circulo UAS.')

        cmds = [wp.command for wp in waypoints]
        has_rp = mission_mod.CMD_DO_RETURN_PATH_START in cmds
        has_ls = mission_mod.CMD_DO_LAND_START in cmds
        if has_rp and has_ls and (cmds.index(mission_mod.CMD_DO_RETURN_PATH_START)
                                  < cmds.index(mission_mod.CMD_DO_LAND_START)):
            self._add('ok', 'DO_RETURN_PATH_START (188) vem antes do '
                            'DO_LAND_START (189).')
        else:
            self._add('erro', 'Ordem DO_RETURN_PATH_START -> DO_LAND_START '
                              'em falta ou invertida: o retorno automatico falha.')

    def check_fence(self):
        """Confirma que ha' uma fence de inclusao e que a missao cabe la' dentro."""
        vehicle_node = getattr(self, self.vehicle)
        fences = self._wait_for(
            lambda: list(vehicle_node.geofence_fences.waypoints), 5.0)
        if not fences:
            self._add('erro', 'Nao ha geofence no FCU. Com FENCE_TYPE = 5 o '
                              'pre-arm FALHA sem poligono de inclusao.')
            return

        kinds = {f.command for f in fences}
        if kinds <= {fence_mod.CMD_FENCE_POLY_INCLUSION,
                     fence_mod.CMD_FENCE_CIRCLE_INCLUSION,
                     fence_mod.CMD_FENCE_RETURN_POINT}:
            self._add('ok', f'Geofence de inclusao com {len(fences)} pontos.')
        else:
            self._add('aviso', f'Geofence com comandos inesperados: {sorted(kinds)}.')

        errors = fence_mod.validate_fence(
            [f for f in fences if f.command != fence_mod.CMD_FENCE_RETURN_POINT])
        for e in errors:
            self._add('erro', 'Fence: ' + e)

    def check_params(self):
        """Le a tabela de parametros do FCU e compara com `artex27.params`."""
        if not self._pull_params():
            self._add('aviso', 'param/pull falhou ou nao respondeu: os valores '
                               'lidos podem estar desatualizados.')
        values = self._fcu_params(sorted(params_mod.ARTEX_PARAMS))

        if all(v is None for v in values.values()):
            self._add('aviso', 'Nao foi possivel ler nenhum parametro do FCU '
                               '(param/get_parameters indisponivel). '
                               'Verificar a mao com `param show` no MAVProxy.')
            return

        _, divergences, missing = params_mod.check_params(values)
        for name, got, want, reason in divergences:
            self._add('erro', f'{name} = {got:g} no FCU, esperado {want:g}. {reason}')
        for name in missing:
            self._add('aviso', f'{name} nao foi lido do FCU.')
        if not divergences and not missing:
            self._add('ok', f'Os {len(values)} parametros da missao conferem.')

    def report(self):
        """Resumo final e riscos abertos. Devolve 0 se nao houver erros."""
        errors = [f for f in self.findings if f[0] == 'erro']
        warnings = [f for f in self.findings if f[0] == 'aviso']
        self.get_logger().info(
            'RESUMO: %d erros, %d avisos, %d verificacoes ok.'
            % (len(errors), len(warnings),
               len([f for f in self.findings if f[0] == 'ok'])))
        self.get_logger().warn('Riscos abertos (nao verificaveis pelo FCU):')
        for title, _ in params_mod.OPEN_RISKS:
            self.get_logger().warn('  * ' + title)
        return 1 if errors else 0

    def run(self):
        """Corre as tres verificacoes e devolve o codigo de saida."""
        if not wait_for_connection(self, [self.vehicle]):
            self.get_logger().error('Sem ligacao ao veiculo.')
            return 1
        self.check_mission()
        self.check_fence()
        if self._p('check_params'):
            self.check_params()
        return self.report()


def main(args=None):
    """Ponto de entrada do `ros2 run artex27 artex_check`."""
    rclpy.init(args=args)
    node = ArtexCheckNode()
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
