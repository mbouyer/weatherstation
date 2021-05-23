# installer for the n2k_driver driver
# Copyright 2021 Manuel Bouyer

from weecfg.extension import ExtensionInstaller


def loader():
    return N2kDriverInstaller()


class N2kDriverInstaller(ExtensionInstaller):
    def __init__(self):
        super(N2kDriverInstaller, self).__init__(
            version="0.6",
            name='n2k_driver',
            description='n2k driver for weewx.',
            author="Manuel Bouyer",
            author_email="",
            config={
                'Station': {
                    'station_type': 'N2kDriver'},
                'N2kDriver': {
                    'interface': 'canlo0',
                    'driver': 'user.n2k_driver'}},
            files=[('bin/user', ['bin/user/n2k_driver.py'])]
        )
