import numpy as np
from copy import deepcopy as dp
from numba import jit, cuda
import matplotlib.pyplot as plt

class move():
    UP = 0
    DOWN = 1
    RIGHT = 2
    LEFT = 3
                     
class QLearning():

    def __init__(self, shape, learning_rate, discount_factor):
        assert 0 < learning_rate < 1, "Learning rate must be between 0 and 1"
        assert 0 < discount_factor < 1, "Discount Factor must be between 0 and 1"

        self.learning_rate = learning_rate
        self.discount_factor = discount_factor
        self.q_table = np.zeros(shape=shape)

    def update(self, s_t, a_t, reward, s_next):
        self.q_table[s_t][a_t] += self.learning_rate * self._get_temporal_difference(s_t, a_t, reward, s_next)
    
    def _get_temporal_difference(self, s_t, a_t, reward, s_next):
        return self._get_temporal_difference_target(s_t, reward, s_next) - self.q_table[s_t][a_t]

    def _get_temporal_difference_target(self, s_t, reward, s_next):
        return reward + self.discount_factor * np.max(self.q_table[s_next]-self.q_table[s_t])

    def get_best_action(self, s_t):
        best_action = np.where(self.q_table[s_t] == self.q_table[s_t].max())
        if len(best_action)>1:
            x = np.random.choice(best_action)
            return x
        else:
            return best_action[0][0]

# @jit(target_backend='cuda')                         
def main():
    r = c = 6
    n = 1
    max_actions = 1000
    shape = (r*c + 1, 4)
    learning_rate = 0.5
    discount_factor = .99
    epsilon = 0.1
    ql = QLearning(shape, learning_rate, discount_factor)
    rw = 0
    start = [1, 1]
    goal = [r-2, c-2]
    successful_rewards = []
    for iter in range(10_000):

        current_pos = dp(start)
        next_pos = dp(current_pos)
        states = [c*current_pos[0] + current_pos[1]]
        actions = []
        rewards = []
        reward = 0
        counter = 0
        for counter in range(max_actions):
            # next_pos = dp(current_pos)
            pos = c*current_pos[0] + current_pos[1]
            if np.random.rand() < epsilon:
                best_action = np.random.randint(4)

                match best_action:
                    case move.UP:
                        next_pos[0] += 1

                    case move.DOWN:
                        next_pos[0] -= 1
                
                    case move.RIGHT:
                        next_pos[1] += 1
                    
                    case move.LEFT:
                        next_pos[1] -= 1
            
            else:
                best_action = ql.get_best_action(pos)
                
                match best_action:
                    case move.UP:
                        next_pos[0] += 1

                    case move.DOWN:
                        next_pos[0] -= 1
                
                    case move.RIGHT:
                        next_pos[1] += 1
                    
                    case move.LEFT:
                        next_pos[1] -= 1
            
            next_pos_converted = c*next_pos[0] + next_pos[1]
            
            if next_pos[0]>=r-1 or next_pos[0]<1 or next_pos[1]>=c-1 or next_pos[1]<1:
                reward += -100
                # print(next_pos)
                states.append(next_pos_converted)
                actions.append(best_action)
                rewards.append(-100)
                if counter > n:
                    states = states[1:]
                    actions = actions[1:]
                    rewards = rewards[1:]
                    ql.update(states[0], actions[0], rewards[0], states[-1])
                # rewards.append(reward)
                break

            if next_pos == goal:
                reward += 1000
                states.append(next_pos_converted)
                actions.append(best_action)
                rewards.append(1000)
                if counter > n:
                    states = states[1:]
                    actions = actions[1:]
                    rewards = rewards[1:]
                    ql.update(states[0], actions[0], rewards[0], states[-1])
                # print(f"Reached Goal with cumulative reward of {reward}")
                rw = max(rw, reward)
                successful_rewards.append(reward)
                break
                
            reward += -1
            states.append(next_pos_converted)
            actions.append(best_action)
            rewards.append(-1)
            if counter > n:
                states = states[1:]
                actions = actions[1:]
                rewards = rewards[1:]
                ql.update(states[0], actions[0], rewards[0], states[-1])
            current_pos = dp(next_pos)
            # counter += 1
        if not iter %1000:
            print(iter)
        while len(states)>2:
            states = states[1:]
            actions = actions[1:]
            rewards = rewards[1:]
            ql.update(states[0], actions[0], rewards[0], states[-1])

    for i in range(len(ql.q_table)):
        print(i, ql.q_table[i])

    plt.plot(range(len(successful_rewards)), successful_rewards)
    #print(successful_rewards.count(max(successful_rewards)))
    # plt.show()
    #print(rw, len(successful_rewards))
    #print("Optimal set of moves: ")
    # return
    start = start[0]*c + start[1]
    end = goal[0]*c + goal[1]
    #print(start, end)
    # return
    path = ""
    while start != end:

        best_move = ql.get_best_action(start)

        match best_move:
            case move.UP:
                path += 'U'
                start += c

            case move.DOWN:
                path += 'D'
                start -= c

            case move.RIGHT:
                path += 'R'
                start += 1

            case move.LEFT:
                path += 'L'
                start -= 1
    print(path)
    with open("action.txt", 'w') as f:
        f.write(path)

        
if __name__ == "__main__":
    main()