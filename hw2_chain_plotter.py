#!/usr/bin/env python3
import rospy
import matplotlib.pyplot as plt
from cs476.msg import Chain2D
import math
import numpy as np


def get_chain_msg():
    """Return a message from the "chain_config" channel.
	
    This function will wait until a message is received.
    """
    rospy.init_node("listener", anonymous=True)
    msg = rospy.wait_for_message("chain_config", Chain2D)
    return msg
    # TODO: Implement this function
    raise NotImplementedError


def plot_chain(config, W, L, D):
    """Plot a 2D kinematic chain A_1, ..., A_m

    @type config: a list [theta_1, ..., theta_m] where theta_1 represents the angle between A_1 and the x-axis,
        and for each i such that 1 < i <= m, \theta_i represents the angle between A_i and A_{i-1}.
    @type W: float, representing the width of each link
    @type L: float, representing the length of each link
    @type D: float, the distance between the two points of attachment on each link
    """

    (joint_positions, link_vertices) = get_link_positions(config, W, L, D)

    fig, ax = plt.subplots()
    plot_links(link_vertices, ax)
    plot_joints(joint_positions, ax)
    ax.axis("equal")
    plt.show()


def plot_links(link_vertices, ax):
    """Plot the links of a 2D kinematic chain A_1, ..., A_m on the axis ax

    @type link_vertices: a list [V_1, ..., V_m] where V_i is the list of [x,y] positions of vertices of A_i
    """

    for vertices in link_vertices:
        x = [vertex[0] for vertex in vertices]
        y = [vertex[1] for vertex in vertices]

        x.append(vertices[0][0])
        y.append(vertices[0][1])
        ax.plot(x, y, "k-", linewidth=2)


def plot_joints(joint_positions, ax):
    """Plot the joints of a 2D kinematic chain A_1, ..., A_m on the axis ax

    @type joint_positions: a list [p_1, ..., p_{m+1}] where p_i is the position [x,y] of the joint between A_i and A_{i-1}
    """
    x = [pos[0] for pos in joint_positions]
    y = [pos[1] for pos in joint_positions]
    ax.plot(x, y, "k.", markersize=10)


def get_link_positions(config, W, L, D):
    """Compute the positions of the links and the joints of a 2D kinematic chain A_1, ..., A_m

    @type config: a list [theta_1, ..., theta_m] where theta_1 represents the angle between A_1 and the x-axis,
        and for each i such that 1 < i <= m, \theta_i represents the angle between A_i and A_{i-1}.
    @type W: float, representing the width of each link
    @type L: float, representing the length of each link
    @type D: float, the distance between the two points of attachment on each link

    @return: a tuple (joint_positions, link_vertices) where
        * joint_positions is a list [p_1, ..., p_{m+1}] where p_i is the position [x,y] of the joint between A_i and A_{i-1}
        * link_vertices is a list [V_1, ..., V_m] where V_i is the list of [x,y] positions of vertices of A_i
    """
    joint_positions = [[0,0]]
    angle = 0
    vert1, vert2, vert3,vert4 = np.array([11,1,1]),np.array([-1,1,1]),np.array([-1,-1,1]),np.array([11,-1,1])
    vert = np.array([ vert1, vert2, vert3, vert4])
    link_vertices= []
    Gmatrix = None
    for i in range(1,len(config)+1):
    	angle = angle + config[i-1]
    	pos = [joint_positions[i-1][0] + D* np.cos(angle), joint_positions[i-1][1] +D* np.sin(angle)]
    	joint_positions.append(pos)
    	#print(joint_positions, pos, len(config))
    	t_matrix = np.array([[np.cos(angle),-(np.sin(angle)),0],[np.sin(angle),np.cos(angle),0],[0,0,1]])
    	vertices=  []
    	
    	if i ==1:
    		for j in range(len(vert)):
    			m= vert[j].reshape((3,1))
    			dot = np.dot(t_matrix,m)
    			vertices.append(dot.reshape((1,3)).tolist()[0][:-1])
    		print(vertices)
    		link_vertices.append(vertices)
    		Gmatrix = t_matrix
    	else:
    		t_matrix = np.array([[np.cos(config[i-1]),-(np.sin(config[i-1])),10],[np.sin(config[i-1]),np.cos(config[i-1]),0],[0,0,1]])
    		Gmatrix = np.dot(Gmatrix,t_matrix)
    		
    		for j in range(len(vert)):
    			m= vert[j].reshape((3,1))
    			dot = np.dot(Gmatrix,m)
    			vertices.append(dot.reshape((1,3)).tolist()[0][:-1])
    		print(vertices)
    		link_vertices.append(vertices)
    return (joint_positions, link_vertices)
    # TODO: Implement this function
    raise NotImplementedError


if __name__ == "__main__":
    chain = get_chain_msg()
    plot_chain(chain.config, chain.W, chain.L, chain.D)
