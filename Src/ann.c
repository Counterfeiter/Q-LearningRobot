/*
 * ann.c
 *
 *  Created on: 22.08.2016
 *      Author: Sebastian Förster
 *      Website: sebstianfoerster86.wordpress.com
 *      License: GPLv2
 */

#include <stdbool.h>
#include <string.h>
#include "stm32f4xx_hal.h"
#include "ann.h"
#include "HC_SR04.h"
#include "servo.h"

#define NORMALIZE_POSITION(__x)		(((float)(__x - SERVO_MINIMUM_ANGLE_US) \
									/ (float)(SERVO_MAXIMUM_ANGLE_US - SERVO_MINIMUM_ANGLE_US)) * 2.0f - 1.0f);
//used for experience reply
#define MAX_BATCH_MEM				100	//google deep mind (Atari) -> 1e6
#define MINI_BATCH_MEM				10	//google deep mind (Atari) -> 32

//FF-ANN train parameter
#define NUM_TRAIN_EPOCHS			30

///steps before update the ann copy
#define ITER_BEFORE_COPY			100

#define DISTANCE_MEASURE_MEDIAN		5

//system parameters
#define NUM_SERVO_MOT				4

//FF parameter
#define NUM_INPUTS					(NUM_SERVO_MOT * 2)
#define NUM_OUTPUTS					(NUM_SERVO_MOT * 3)

//reward network to walk forward or backward?
#define TRAIN_FORWARD				1

struct experience_reply_s
{
	//0 = move forward, 1 = move backward, 2 = stop
	uint8_t servo_actions[NUM_SERVO_MOT];

	//inputs, inputs t+1
	fann_type inputs[NUM_INPUTS];
	fann_type inputs_tp1[NUM_INPUTS];

	//reward that was calc. for each motor
	fann_type reward[NUM_SERVO_MOT];
};

//save old servo pos. values for the FF-ANN input
fann_type old_pos[NUM_INPUTS - NUM_SERVO_MOT] = { 0.0f };
//save old distance to calc the difference of the done step
uint16_t old_distance;

static void ann_getInputPositions(fann_type *input_vec);
void ann_set_new_motor(int output);
static void ann_displayVector(char *headline, fann_type *vec, int size);
static void ann_displayInputOutputFANN(fann_type *inputs, int num_inputs, fann_type *outputs, int num_outputs);
static int 	ann_getMaxQandAction(int servo_mot, fann_type *ann_output_vec, fann_type *maxQ);
static float median(int n, int x[]);



//use fann lib and qlerning to train a ann to set desired led pattern
//source is ported from here -> http://outlace.com/Reinforcement-Learning-Part-3/
void ann_start_qlearning(int epochs, float gamma, float epsilon, int steps_ms)
{
	//google deep mind (Atari) -> gamma = 0.99, epsilon 1.0 to 0.1

	//create ann
	struct fann *ann = fann_create_standard(4, NUM_INPUTS, 20, 12, NUM_OUTPUTS);
	//some tests
	//struct fann *ann = fann_create_standard(3, NUM_INPUTS, 25, NUM_OUTPUTS);
	//struct fann *ann = fann_create_standard(2, NUM_INPUTS, NUM_OUTPUTS);
	//struct fann *ann = fann_create_standard(5, NUM_INPUTS, 10, 15, 12, NUM_OUTPUTS);
	//struct fann *ann = fann_create_standard(5, num_inputs, 40, 25, 15, num_outputs);

	//stepwise is more then two times faster
	//use symmetric to deal with -1.0 and 1.0 (or normal for 0.0 to 1.0)
	fann_set_activation_function_hidden(ann, FANN_SIGMOID_SYMMETRIC_STEPWISE);
	fann_set_activation_function_output(ann, FANN_SIGMOID_SYMMETRIC_STEPWISE);

	fann_set_learning_momentum(ann, 0.95); 	//google deep mind (Atari) -> 0.95
	fann_set_learning_rate(ann, 0.1);		//google deep mind (Atari) -> 0.00025

	fann_set_training_algorithm(ann, FANN_TRAIN_BATCH);

	/*int cnt_copy = 0;
	//make a copy like in googles deep mind patent
	struct fann *ann_cpy = fann_copy(ann);
	//set all weights zero (not random)
	for(int i = 0; i < ann_cpy->total_connections; i++)
	{
		ann_cpy->weights[i] = 0.0f;
	}*/

	//register two buffers and two pointers, to swap the buffers with input values fast
	fann_type new_inputs[NUM_INPUTS];
	fann_type old_inputs[NUM_INPUTS];
	fann_type *new_in_p = new_inputs;
	fann_type *old_in_p = old_inputs;

	//setup reply mems
	struct fann_train_data *train_data = fann_create_train(MINI_BATCH_MEM, NUM_INPUTS, NUM_OUTPUTS);
	struct experience_reply_s batch_mem[MAX_BATCH_MEM];

	//are the batch buffer full? counter...
	int batch_pos = 0;

	//store the maximum done servo move to reward the network correct with the given tick rate
	float max_possible_servo_move = 0.01f;

	//setup initial pos.
	for(int i = 0; i < NUM_SERVO_MOT; i++)
		servo_set_value(i, 1450, 500, 200);

	HAL_Delay(1000);

	//save start distance
	old_distance = hcsr04_getLastDistance_mm();

	//get position
	ann_getInputPositions(new_in_p);

	for(int q = 0; q < epochs; q++ )
	{

		//actions could be forward (0) or backward (1) for each servo motor
		uint8_t actions[4] = {0, 0, 0, 0};

		//switch pointer -> new data, to old data
		fann_type *temp = old_in_p;
		old_in_p = new_in_p;
		new_in_p = temp;

		//run ann network
		fann_type *qval_p = fann_run(ann, old_in_p);

		//copy output data -> later use
		//because qval_p is just a pointer no allocated mem
		fann_type qval[NUM_OUTPUTS];
		for(int x = 0; x < NUM_OUTPUTS; x++)
		{
			qval[x] = qval_p[x];
		}

		//if epsilon is high, we use random actions
		//we should make all possible actions to train this
		if((float)rand() / RAND_MAX < epsilon)
		{
			for(int x = 0; x < NUM_SERVO_MOT; x++)
				actions[x] = rand() % 3; // action is 0, 1 or 2

			printf("Use RANDOM action\n");
		} else {
			//search the maximum in the output array
			//for each servo pair -> forward or backward is maximum?
			for(int x = 0; x < NUM_SERVO_MOT; x++)
				actions[x] = ann_getMaxQandAction(x, qval_p, NULL);

			printf("Use WINNER action\n");
		}

		//do the action that the ann predict...
		for(int x = 0; x < NUM_SERVO_MOT; x++)
			ann_set_new_motor(actions[x] * NUM_SERVO_MOT + x);

		//let it move
		//HAL_Delay(steps_ms);
		while(!servo_allInPos());

		//fill array with new state
		ann_getInputPositions(new_in_p);



		//////////////CALC REWARD //////////////////////
		int med_div[DISTANCE_MEASURE_MEDIAN];
		int iii;
		uint32_t new_dist;
		int div = 0;
		for( iii = 0; iii < DISTANCE_MEASURE_MEDIAN; iii++)
		{
			//a bit more time to measure the actual distance correct
			HAL_Delay(25);
			//check distance
			new_dist = hcsr04_getLastDistance_mm();

			//discard measurement errors
			if(new_dist != 0)
			{
	#ifdef TRAIN_FORWARD
				div = (int)old_distance - new_dist;
	#else
				div = (int)new_dist - old_distance;
	#endif
				if(div < 70 && div > -70)
				{
					med_div[iii] = div;
				} else {
					break;
				}
			} else {
				break;
			}
		}

		//discard any measure errors
		if(iii != DISTANCE_MEASURE_MEDIAN)
		{
			continue;
		} else {
			div = (int) (median(DISTANCE_MEASURE_MEDIAN, med_div) + 0.5f);
#ifdef TRAIN_FORWARD
			old_distance -= div;
#else
			old_distance += div;
#endif
			printf("Difference distance: %d mm\n", div);
		}




		//calc reward because of moving forward/backward
		fann_type reward[NUM_SERVO_MOT] = {0.0 , 0.0, 0.0, 0.0 };

		for(int x = 0; x < NUM_SERVO_MOT; x++)
		{
			reward[x] = (float)div / 70.0f;

			//if(reward[x] > 0.0f) reward[x] *= 2.0f;
			//printf("Reward %f\n", reward[x]);
		}

		//printf("Max move: %f\n", max_possible_servo_move);

		//calc a reward because the leg was moving
		//this should avoid a robot that is satisfied with stopping legs
		/*for(int x = 0; x < NUM_SERVO_MOT; x++)
		{
			float move_dist = fabsf(new_in_p[x] - new_in_p[x + NUM_SERVO_MOT]); //max could be 2.0
			max_possible_servo_move = (move_dist > max_possible_servo_move) ? move_dist : max_possible_servo_move;
			//moving the leg lesser then half way -> add negative reward
			//moving the leg more then the half way -> add positive reward
			//Divider => how much count only the leg moving compared to the distance
			reward[x] += ((move_dist - max_possible_servo_move / 2.0f) / max_possible_servo_move) / 5.0f;

			//display reward decision
			printf("Reward %d: %.2f -> moved %.2f\n", x, reward[x], move_dist);
		}*/


		/*for(int x = 0; x < NUM_SERVO_MOT; x++)
		{
			// clip the reward like in google deep mind paper
			// because the output if the network could only be -1.0 to 1.0
			reward[x] = reward[x] < -1.0f ? -1.0f : reward[x];
			reward[x] = reward[x] > 1.0f ? 1.0f : reward[x];

		}*/




		//////////////CALC REWARD END//////////////////////

		//use this delay to study the output printfs (debug)
		//HAL_Delay(2000);

		//display only output neurons
		ann_displayInputOutputFANN(old_in_p, 0, qval, NUM_OUTPUTS);

		//check if batch mem is filled the first time
		if(batch_pos >= MAX_BATCH_MEM)
		{
			//if the list is filled once a time completely
			//we start to set new data at random positions
			//save outputs and inputs
			int batch_rand_pos = rand() % MAX_BATCH_MEM;
			memcpy(batch_mem[batch_rand_pos].inputs, old_in_p, NUM_INPUTS * sizeof(fann_type));
			memcpy(batch_mem[batch_rand_pos].inputs_tp1, new_in_p, NUM_INPUTS * sizeof(fann_type));
			memcpy(batch_mem[batch_rand_pos].reward, reward, NUM_SERVO_MOT * sizeof(fann_type));
			memcpy(batch_mem[batch_rand_pos].servo_actions, actions, NUM_SERVO_MOT * sizeof(uint8_t));

			/////////////////// experience reply //////////////////////
			//sample a random set of MAX_BATCH_MEM to MIN_BATCH_MEM
			for(int y = 0; y < MINI_BATCH_MEM; y++)
			{
				int mini_pos = rand() % MAX_BATCH_MEM;

				//run the old copy of the ann with new inputs
				//fann_type *newQ = fann_run(ann_cpy, batch_mem[mini_pos].inputs_tp1);
				fann_type *newQ = fann_run(ann, batch_mem[mini_pos].inputs_tp1);

				//search the maximum in newQ
				fann_type maxQ[NUM_SERVO_MOT];
				for(int x = 0; x < NUM_SERVO_MOT; x++)
				{
					ann_getMaxQandAction(x, newQ, &maxQ[x]);
				}

				fann_type *oldQ = fann_run(ann, batch_mem[mini_pos].inputs);

				//set the old values as train data (output) - use new max in equation
				for(int x = 0; x < NUM_SERVO_MOT; x++)
				{
					oldQ[batch_mem[mini_pos].servo_actions[x] * NUM_SERVO_MOT + x] = (batch_mem[mini_pos].reward[x] < 0.1) ? \
							(batch_mem[mini_pos].reward[x] + (gamma * maxQ[x])) : batch_mem[mini_pos].reward[x];
				}

				//ann_displayVector("Desired Outputs", oldQ, num_outputs);

				//store the data in mini batch fann train file
				memcpy(train_data->input[y], batch_mem[mini_pos].inputs, NUM_INPUTS * sizeof(fann_type));
				memcpy(train_data->output[y], oldQ, NUM_OUTPUTS * sizeof(fann_type));
			}

			//train the data sets
			fann_train_on_data(ann, train_data, NUM_TRAIN_EPOCHS, NUM_TRAIN_EPOCHS, 0.001);

			/////////////////// experience reply end //////////////////////
		} else {
			//save outputs and inputs
			memcpy(batch_mem[batch_pos].inputs, old_in_p, NUM_INPUTS * sizeof(fann_type));
			memcpy(batch_mem[batch_pos].inputs_tp1, new_in_p, NUM_INPUTS * sizeof(fann_type));
			memcpy(batch_mem[batch_pos].reward, reward, NUM_SERVO_MOT * sizeof(fann_type));
			memcpy(batch_mem[batch_pos].servo_actions, actions, NUM_SERVO_MOT * sizeof(uint8_t));

			//this function use always backpropagation algorithm!
			//fann_set_training_algorithm has no effect!
			//or same as fann_set_training_algorithm = incremental and train epoch
			//train ann   , input, desired outputs
			//train only single data -> catastrophic forgetting could happen in the first MAX_BATCH_MEM moves
			fann_train(ann, old_in_p, qval);
			batch_pos++;
		}

		//Decrease epsilon
		if(epsilon > 0.1)
			epsilon -= ( 1.0 / epochs );

		/*if(cnt_copy++ > ITER_BEFORE_COPY)
		{
			cnt_copy = 0;
			//copy weights
			//see https://storage.googleapis.com/deepmind-data/assets/papers/DeepMindNature14236Paper.pdf
			for(int i = 0; i < ann_cpy->total_connections; i++)
			{
				ann_cpy->weights[i] = ann->weights[i];
			}
		}*/

		//we are at the end of the train track?
		//blinker and say: "set me back in the middle"
		if(new_dist > 270 || new_dist < 40)
		{
			while(!HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin))
			{
				HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
				HAL_Delay(50);
			}
			HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, 0);
			HAL_Delay(1000);
			old_distance = hcsr04_getLastDistance_mm();

			continue;
		}
	}

	//mark that we go to the execution state
	//set all LEDs
	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, 1);
	HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, 1);
	HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, 1);
	HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, 1);

	printf("\n\n\n");
	fann_save(ann, "stdout");
	printf("\n\n\n");

	while(!HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin));

	HAL_Delay(1000);

	ann_executing(ann, steps_ms);

}

void ann_executing(struct fann *ann, int steps_ms)
{
	fann_type new_inputs[NUM_INPUTS];

	int cnt_rnd_moves = 0;
	//walk after training
	//execute forever
	while(1)
	{
		//get position
		ann_getInputPositions(new_inputs);
		//execute ann
		fann_type *exec_out = fann_run(ann, new_inputs);

		ann_displayInputOutputFANN(NULL, 0, exec_out, NUM_OUTPUTS);

		float move_dist = 0.0f;

		//do the action that the ann predict...
		//search the maximum in the output array
		for(int x = 0; x < NUM_SERVO_MOT; x++)
		{

			ann_set_new_motor(ann_getMaxQandAction(x, exec_out, NULL) * NUM_SERVO_MOT + x);

			move_dist += fabsf(new_inputs[x] - new_inputs[x + NUM_SERVO_MOT]);
		}

		//push the network to move in a loop
		if(move_dist <= 0.0)
		{

			if(cnt_rnd_moves++ > 3)
			{
				cnt_rnd_moves = 0;
				printf("Robot stopping! Use random move!\n");
				for(int x = 0; x < NUM_SERVO_MOT; x++)
				{
					ann_set_new_motor((rand() % 3) * NUM_SERVO_MOT + x);
				}
			}
		}

		//let it move
		HAL_Delay(steps_ms);
		while(!servo_allInPos());

		//if button pressed
		if(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin))
		{
			while(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin));
			return;
		}
	}
}

static int ann_getMaxQandAction(int servo_mot, fann_type *ann_output_vec, fann_type *maxQ)
{
	int forward = servo_mot;
	int backward = servo_mot + NUM_SERVO_MOT;
	int stop = servo_mot + 2 * NUM_SERVO_MOT;

	//printf("Servo decision: F: %f   B: %f   S: %f\n", ann_output_vec[forward], ann_output_vec[backward], ann_output_vec[stop]);

	if(ann_output_vec[forward] > ann_output_vec[backward] && ann_output_vec[forward] > ann_output_vec[stop])
	{
		if(maxQ != NULL)
			*maxQ =  ann_output_vec[forward];
		return 0; //forward
	} else if(ann_output_vec[stop] > ann_output_vec[backward] && ann_output_vec[stop] > ann_output_vec[forward])
	{
		if(maxQ != NULL)
			*maxQ =  ann_output_vec[stop];
		return 2; //stop
	}

	if(maxQ != NULL)
		*maxQ =  ann_output_vec[backward];
	return 1; //backward
}

static void ann_getInputPositions(fann_type *input_vec)
{
	int i;
	//load new positions
	for(i = 0; i < NUM_SERVO_MOT; i++)
	{
		input_vec[i] = NORMALIZE_POSITION(servo_getpos(i));
		//printf("Input %d: %.1f\n", i, input_vec[i]);
	}

	//load old positions
	for(; i < NUM_INPUTS; i++)
	{
		input_vec[i] = old_pos[i - NUM_SERVO_MOT];
		old_pos[i - NUM_SERVO_MOT] = input_vec[i - NUM_SERVO_MOT];
		//printf("Input %d: %.1f\n", i, input_vec[i]);
	}
}

void ann_set_new_motor(int output)
{

	//move motor to three different positions
	int position = (output > 7) ? SERVO_MAXIMUM_ANGLE_US : 1450;
	position = (output <= 3) ? SERVO_MINIMUM_ANGLE_US : position;

	//printf("Motor: %d Set Position: %d\n", output % 4, position);
	servo_set_value(output % 4, position, UINT16_MAX, 400);
}

//////////////// DISPLAY STUFF ///////////////

static void ann_displayVector(char *headline, fann_type *vec, int size)
{
	printf("%s\n", headline);
	printf("-------\n");
	int i;
	//load new positions
	for(i = 0; i < size ; i++)
	{
		printf("Vector %d: %.2f\n", i, vec[i]);
	}
	printf("-------\n");
}

static void ann_displayInputOutputFANN(fann_type *inputs, int num_inputs, fann_type *outputs, int num_outputs)
{
	for(int i=0; i < num_inputs; i++)
		printf("%f ", inputs[i]);
	for(int i=0; i < num_outputs; i++)
			printf("%f ", outputs[i]);
	printf("\n");
}


//https://en.wikiversity.org/wiki/C_Source_Code/Find_the_median_and_mean
static float median(int n, int x[]) {
    float temp;
    int i, j;
    // the following two loops sort the array x in ascending order
    for(i=0; i<n-1; i++) {
        for(j=i+1; j<n; j++) {
            if(x[j] < x[i]) {
                // swap elements
                temp = x[i];
                x[i] = x[j];
                x[j] = temp;
            }
        }
    }

    if(n%2==0) {
        // if there is an even number of elements, return mean of the two elements in the middle
        return((x[n/2] + x[n/2 - 1]) / 2.0);
    } else {
        // else return the element in the middle
        return x[n/2];
    }
}
