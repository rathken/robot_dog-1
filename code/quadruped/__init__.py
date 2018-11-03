import logging

from gym.envs.registration import register

logger = logging.getLogger(__name__)

register(
    id='quadruped-v0',
    entry_point='quadruped.envs:QuadrupedEnv'
)


