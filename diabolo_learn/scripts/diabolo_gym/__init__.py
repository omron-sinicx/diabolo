from gym.envs.registration import register

register(
    id='diabolo-v0',
    entry_point='diabolo_gym.diabolo_env:DiaboloEnv'
)
