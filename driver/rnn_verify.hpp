#ifndef GUARD_MIOPEN_RNN_VERIFY_HPP
#define GUARD_MIOPEN_RNN_VERIFY_HPP

#include <math.h>
#include <cassert>

int sumv(std::vector<int>& x)
{
	int sum = 0;
	for (int i = 0; i < x.size(); i++)
	{
		sum += x[i];
	}
	return sum;
}

float activfunc(float x, int actvf)
{
	switch (actvf)
	{
		case 0:  // ReLU
		{
			return max(x, 0);
		}
		case 1:  // tanh
		{
			return tanh(x);
		}
	}
}

float dervactivfunc(float x, int actvf)
{
	switch (actvf)
	{
		case 0:  // ReLU
		{
			return (x > 0 ? 1 : 0 );
		}
		case 1:  // tanh
		{
			return 1/ cosh(x) / cosh(x);
		}
	}
}

template <typename T>
void RunRNNForwardCPUVerify(std::vector<T>& in,
	std::vector<T>& wei, // [ input_state_weight_trans  hidden_state_weight0_trans input1_trans hidden1_trans ... output_weight; bidirectional reversed weights ]
	std::vector<T>& hy_host, // current/final hidden state
	std::vector<T>& hx, // initial hidden state
	std::vector<T>& out_host,
	std::vector<int>& in_n, // input batch size
	int in_h, // input data length
	int seqLength, // Number of iterations to unroll over
	bool bidirection, // whether using bidirectional net
	bool biased, // whether using bias
	int hy_d, // 1 by numlayer (number of stacks of hidden layers) for unidirection, 2 by numlayer for bidirection
	int hy_n, // equal to input batch size in_n[0]
	int hy_h, // hidden state number
	std::vector<int>& out_n, // equals in_n
	int out_h;  // 1 by hy_h related function for unidirection, 2 by hy_h related function for bidirection
    std::vector<T>& rsvspace
)
{
	int batch_n = sumvc(in_n);
	T * hid_state = new T[hy_d * batch_n * hy_h];
	memset(hid_state, 0, hy_d * batch_n * hy_h * sizeof(T));

	T * out_state = new T[batch_n * out_h];
	memset(out_state, 0, batch_n * out_h * sizeof(T));

	int numlayer = bidirection ? hy_d / 2 : hy_d;
	int out_dim = bidirection ? out_h / 2 : out_h;
	int bacc; // accumulation of batch
	int bi = bidirection ? 2 : 1;
	int squash = cudnnRNNMode_t == CUDNN_RNN_RELU ? 0 : 1;

	for (int li = 0; li < numLayer; li++)
	{
		bacc = 0;
		for (int ti = 0; ti < seqLength; ti++)
		{
			for (int bs = 0; bs < in_n[ti]; bs++)
			{
				for (int h = 0; h < hy_h; h++)
				{
					if (li == 0)
					{
						// from input
						for (int w = 0; w < in_h; w++)
						{
							hid_state[li * batch_n * hy_h * bi + (bacc + bs) * hy_h + h] += wei[w * hy_h + h] * in[(bacc + bs) * in_h + w];
						}

						// from previous state
						for (int w = 0; w < hy_h; w++)
						{
							if (ti == 0)
							{
								hid_state[li * batch_n * hy_h * bi + (bacc + bs) * hy_h + h] += wei[in_h * hy_h + w * hy_h + h] * hx[li * bi * in_n[0] * hy_h + bs * hy_h + w];
							}
							else
							{
								hid_state[li * batch_n * hy_h * bi + (bacc + bs) * hy_h + h] += wei[in_h * hy_h + w * hy_h + h] * hy_host[li * bi * in_n[0] * hy_h + bs * hy_h + w];
							}
						}

						//from bias
						if (biased)
						{
							hid_state[li * batch_n * hy_h * bi + (bacc + bs) * hy_h + h] += wei[((in_h + hy_h + out_h) * bi + (bi * hy_h + hy_h) * bi * (numLayer - 1)) * hy_h + h];
							hid_state[li * batch_n * hy_h * bi + (bacc + bs) * hy_h + h] += wei[((in_h + hy_h + out_h) * bi + (bi * hy_h + hy_h) * bi * (numLayer - 1)) * hy_h + hy_h + h];
						}
					}
					else
					{
						// from input
						for (int w = 0; w < hy_h; w++)
						{
							hid_state[li * batch_n * hy_h * bi + (bacc + bs) * hy_h + h] += wei[bi * (in_h + hy_h) * hy_h + (li - 1) * bi * (bi * hy_h + hy_h) * hy_h + w * hy_h + h] * activfunc(hid_state[(li - 1) * batch_n * hy_h * bi + (bacc + bs) * hy_h + w], squash);
							if (bidirection)
							{
								hid_state[li * batch_n * hy_h * bi + (bacc + bs) * hy_h + h] += wei[bi * (in_h + hy_h) * hy_h + (li - 1) * bi * (bi * hy_h + hy_h) * hy_h + hy_h * hy_h + w * hy_h + h] * activfunc(hid_state[(li - 1) * batch_n * hy_h * bi + batch_n * hy_h + (bacc + bs) * hy_h + w], squash);
							}
						}

						// from previous state
						for (int w = 0; w < hy_h; w++)
						{
							if (ti == 0)
							{
								hid_state[li * batch_n * hy_h * bi + (bacc + bs) * hy_h + h] += wei[bi * (in_h + hy_h) * hy_h + (li - 1) * bi * (bi * hy_h + hy_h) * hy_h + bi * hy * hy + w * hy_h + h] * hx[li * bi * in_n[0] * hy_h + bs * hy_h + w];
							}
							else
							{
								hid_state[li * batch_n * hy_h * bi + (bacc + bs) * hy_h + h] += wei[bi * (in_h + hy_h) * hy_h + (li - 1) * bi * (bi * hy_h + hy_h) * hy_h + bi * hy * hy + w * hy_h + h] * hy_host[li * bi * in_n[0] * hy_h + bs * hy_h + w];
							}
						}

						//from bias
						if (biased)
						{
							hid_state[li * batch_n * hy_h * bi + (bacc + bs) * hy_h + h] += wei[((in_h + hy_h + out_h) * bi + (bi * hy_h + hy_h) * bi * (numLayer - 1)) * hy_h + bi * 2 * hy_h + bi * (li - 1) * (bi + 1) * hy_h + h];
							if (bidirection)
							{
								hid_state[li * batch_n * hy_h * bi + (bacc + bs) * hy_h + h] += wei[((in_h + hy_h + out_h) * bi + (bi * hy_h + hy_h) * bi * (numLayer - 1)) * hy_h + bi * 2 * hy_h + bi * (li - 1) * (bi + 1) * hy_h + hy_h + h];
							}
							hid_state[li * batch_n * hy_h * bi + (bacc + bs) * hy_h + h] += wei[((in_h + hy_h + out_h) * bi + (bi * hy_h + hy_h) * bi * (numLayer - 1)) * hy_h + bi * 2 * hy_h + bi * (li - 1) * (bi + 1) * hy_h + bi * hy_h + h];
						}
					}

					hy_host[li * bi * in_n[0] * hy_h + bs * hy_h + h] = activfunc(hid_state[li * batch_n * hy_h * bi + (bacc + bs) * hy_h + h], squash);  // squash_func

					rsvspace[li * batch_n * hy_h * bi + (bacc + bs) * hy_h + h] = hid_state[li * batch_n * hy_h * bi + (bacc + bs) * hy_h + h];
				}
			}
			bacc += in_n[ti];
		}

		if (bidirection)
		{
			bacc = batch_n;
			for (int ti = seqLength - 1; ti >= 0; ti--)
			{
				bacc -= in_n[ti];
				for (int bs = 0; bs < in_n[ti]; bs++)
				{
					for (int h = 0; h < hy_h; h++)
					{
						if (li == 0)
						{
							// from input
							for (int w = 0; w < in_h; w++)
							{
								hid_state[li * batch_n * hy_h * bi + batch_n * hy_h + (bacc + bs) * hy_h + h] += wei[(in_h + hy_h) * hy_h + w * hy_h + h] * in[(bacc + bs) * in_h + w];
							}

							// from previous state
							for (int w = 0; w < hy_h; w++)
							{
								if (ti == seqLength - 1)
								{
									hid_state[li * batch_n * hy_h * bi + batch_n * hy_h + (bacc + bs) * hy_h + h] += wei[(in_h + hy_h) * hy_h + in_h * hy_h + w * hy_h + h] * hx[li * bi * in_n[0] * hy_h + in_n[0] * hy_h + bs * hy_h + w];
								}
								else
								{
									hid_state[li * batch_n * hy_h * bi + batch_n * hy_h + (bacc + bs) * hy_h + h] += wei[(in_h + hy_h) * hy_h + in_h * hy_h + w * hy_h + h] * hy_host[li * bi * in_n[0] * hy_h + in_n[0] * hy_h + bs * hy_h + w];
								}
							}

							//from bias
							if (biased)
							{
								hid_state[li * batch_n * hy_h * bi + batch_n * hy_h + (bacc + bs) * hy_h + h] += wei[((in_h + hy_h + out_h) * bi + (bi * hy_h + hy_h) * bi * (numLayer - 1)) * hy_h + 2 * hy_h + h];
								hid_state[li * batch_n * hy_h * bi + batch_n * hy_h + (bacc + bs) * hy_h + h] += wei[((in_h + hy_h + out_h) * bi + (bi * hy_h + hy_h) * bi * (numLayer - 1)) * hy_h + 2 * hy_h + hy_h + h];
							}
						}
						else
						{
							// from input
							for (int w = 0; w < hy_h; w++)
							{
								hid_state[li * batch_n * hy_h * bi + batch_n * hy_h + (bacc + bs) * hy_h + h] += wei[bi * (in_h + hy_h) * hy_h + (li - 1) * bi * (bi * hy_h + hy_h) * hy_h + (bi * hy_h + hy_h) * hy_h + w * hy_h + h] * activfunc(hid_state[(li - 1) * batch_n * hy_h * bi + (bacc + bs) * hy_h + w], squash);

								hid_state[li * batch_n * hy_h * bi + batch_n * hy_h + (bacc + bs) * hy_h + h] += wei[bi * (in_h + hy_h) * hy_h + (li - 1) * bi * (bi * hy_h + hy_h) * hy_h + (bi * hy_h + hy_h) * hy_h + hy_h * hy_h + w * hy_h + h] * activfunc(hid_state[(li - 1) * batch_n * hy_h * bi + batch_n * hy_h + (bacc + bs) * hy_h + w], squash);
							}

							// from previous state
							for (int w = 0; w < hy_h; w++)
							{
								if (ti == seqLength - 1)
								{
									hid_state[li * batch_n * hy_h * bi + batch_n * hy_h + (bacc + bs) * hy_h + h] += wei[bi * (in_h + hy_h) * hy_h + (li - 1) * bi * (bi * hy_h + hy_h) * hy_h + (bi * hy_h + hy_h) * hy_h + bi * hy_h * hy_h + w * hy_h + h] * hx[li * bi * in_n[0] * hy_h + in_n[0] * hy_h + bs * hy_h + w];
								}
								else
								{
									hid_state[li * batch_n * hy_h * bi + batch_n * hy_h + (bacc + bs) * hy_h + h] += wei[bi * (in_h + hy_h) * hy_h + (li - 1) * bi * (bi * hy_h + hy_h) * hy_h + (bi * hy_h + hy_h) * hy_h + bi * hy_h * hy_h + w * hy_h + h] * hy_host[li * bi * in_n[0] * hy_h + in_n[0] * hy_h + bs * hy_h + w];
								}
							}

							//from bias
							if (biased)
							{
								hid_state[li * batch_n * hy_h * bi + batch_n * hy_h + (bacc + bs) * hy_h + h] += wei[((in_h + hy_h + out_h) * bi + (bi * hy_h + hy_h) * bi * (numLayer - 1)) * hy_h + bi * 2 * hy_h + bi * (li - 1) * (bi + 1) * hy_h + (bi + 1) * hy_h + h];
								hid_state[li * batch_n * hy_h * bi + batch_n * hy_h + (bacc + bs) * hy_h + h] += wei[((in_h + hy_h + out_h) * bi + (bi * hy_h + hy_h) * bi * (numLayer - 1)) * hy_h + bi * 2 * hy_h + bi * (li - 1) * (bi + 1) * hy_h + (bi + 1) * hy_h + hy_h + h];
								hid_state[li * batch_n * hy_h * bi + batch_n * hy_h + (bacc + bs) * hy_h + h] += wei[((in_h + hy_h + out_h) * bi + (bi * hy_h + hy_h) * bi * (numLayer - 1)) * hy_h + bi * 2 * hy_h + bi * (li - 1) * (bi + 1) * hy_h + (bi + 1) * hy_h + bi * hy_h + h];
							}
						}

						hy_host[li * bi * in_n[0] * hy_h + in_n[0] * hy_h + bs * hy_h + h] = activfunc(hid_state[li * batch_n * hy_h * bi + batch_n * hy_h + (bacc + bs) * hy_h + h], squash);  // squash_func

						rsvspace[li * batch_n * hy_h * bi + batch_n * hy_h + (bacc + bs) * hy_h + h] = hid_state[li * batch_n * hy_h * bi + batch_n * hy_h + (bacc + bs) * hy_h + h];
					}
				}
			}
		}
	}

	// output
	bacc = 0;
	for (int ti = 0; ti < seqLength; ti++)
	{
		for (int bs = 0; bs < in_n[ti]; bs++)
		{
			for (int w = 0; w < out_dim; w++)
			{
				for (int h = 0; h < hy_h; h++)
				{
					out_state[(bacc + bs) * out_h + w] += wei[bi * (in_h + hy_h) * hy_h + (numLayer - 1) * bi * (bi * hy_h + hy_h) * hy_h + w * hy_h + h] * activfunc(hid_state[(numLayer - 1) * batch_n * hy_h * bi + (bacc + bs) * hy_h + h], squash);
					if (bidirection)
					{
						out_state[(bacc + bs) * out_h + w] += wei[bi * (in_h + hy_h) * hy_h + (numLayer - 1) * bi * (bi * hy_h + hy_h) * hy_h + 2 * out_dim * hy_h + w * hy_h + h] * activfunc(hid_state[(numLayer - 1) * batch_n * hy_h * bi + batch_n * hy_h + (bacc + bs) * hy_h + h], squash);

						out_state[(bacc + bs) * out_h + out_dim + w] += (wei[bi * (in_h + hy_h) * hy_h + (numLayer - 1) * bi * (bi * hy_h + hy_h) * hy_h + out_dim * hy_h + w * hy_h + h] * activfunc(hid_state[(numLayer - 1) * batch_n * hy_h * bi + (bacc + bs) * hy_h + h], squash)
							+ wei[bi * (in_h + hy_h) * hy_h + (numLayer - 1) * bi * (bi * hy_h + hy_h) * hy_h + 3 * out_dim * hy_h + w * hy_h + h] * activfunc(hid_state[(numLayer - 1) * batch_n * hy_h * bi + batch_n * hy_h + (bacc + bs) * hy_h + h], squash));
					}

					//from bias
					if (biased)
					{
						out_state[(bacc + bs) * out_h + w] += wei[((in_h + hy_h + out_h) * bi + (bi * hy_h + hy_h) * bi * (numLayer - 1)) * hy_h + bi * 2 * hy_h + bi * (bi + 1) * (numLayer - 1) * hy_h + w];
						if (bidirection)
						{
							out_state[(bacc + bs) * out_h + w] += wei[((in_h + hy_h + out_h) * bi + (bi * hy_h + hy_h) * bi * (numLayer - 1)) * hy_h + bi * 2 * hy_h + bi * (bi + 1) * (numLayer - 1) * hy_h + 2 * out_dim + w];
							out_state[(bacc + bs) * out_h + out_dim + w] += (wei[((in_h + hy_h + out_h) * bi + (bi * hy_h + hy_h) * bi * (numLayer - 1)) * hy_h + bi * 2 * hy_h + bi * (bi + 1) * (numLayer - 1) * hy_h + out_dim + w]
								+ wei[((in_h + hy_h + out_h) * bi + (bi * hy_h + hy_h) * bi * (numLayer - 1)) * hy_h + bi * 2 * hy_h + bi * (bi + 1) * (numLayer - 1) * hy_h + 3 * out_dim + w]);
						}
					}
				}

				out_host[(bacc + bs) * out_h + w] = out_state[(bacc + bs) * out_h + w];
				if (bidirection)
				{
					out_host[(bacc + bs) * out_h + out_dim + w] = out_state[(bacc + bs) * out_h + out_dim + w];
				}
			}
		}
		bacc += in_n[ti];
	}
}


template <typename T>
void RunRNNBackwardDataCPUVerify(std::vector<T>& din_host,
	std::vector<T>& wei, // [ input_state_weight_trans  hidden_state_weight0_trans input1_trans hidden1_trans ... output_weight; bidirectional reversed weights ]
	std::vector<T>& dhy, // current/final hidden state
	std::vector<T>& dhx_host,
	std::vector<T>& hx, // initial hidden state
	std::vector<T>& out,
	std::vector<T>& dout,
	std::vector<int>& in_n, // input batch size
	int in_h, // input data length
	int seqLength, // Number of iterations to unroll over
	bool bidirection, // whether using bidirectional net
	bool biased, // whether using bias
	int hy_d, // 1 by numlayer (number of stacks of hidden layers) for unidirection, 2 by numlayer for bidirection
	int hy_n, // equal to input batch size in_n[0]
	int hy_h, // hidden state number
	std::vector<int>& out_n, // equals in_n
	int out_h;  // 1 by hy_h related function for unidirection, 2 by hy_h related function for bidirection
	std::vector<T>& rsvspace;
	std::vector<T>& wkspace
)
{
	int batch_n = sumvc(in_n);
	T * dh_state = new T[hy_d * batch_n * hy_h];
	memset(dh_state, 0, hy_d * batch_n * hy_h * sizeof(T));

	T * din_state = new T[batch_n * in_h];
	memset(din_state, 0, batch_n * in_h * sizeof(T));

	int numlayer = bidirection ? hy_d / 2 : hy_d;
	int out_dim = bidirection ? out_h / 2 : out_h;
	int bacc; // accumulation of batch
	int bi = bidirection ? 2 : 1;
	int squash = cudnnRNNMode_t == CUDNN_RNN_RELU ? 0 : 1;


	for (int li = numLayer -1 ; li >= 0; li++)
	{
		bacc = batch_n;
		for (int ti = seqLength - 1; ti >= 0; ti--)
		{
			bacc -= in_n[ti];
			for (int bs = 0; bs < in_n[ti]; bs++)
			{
				for (int h = 0; h < hy_h; h++)
				{
					if (li == numLayer - 1)
					{
						// from doutput
						for (int w = 0; w < out_h; w++)
						{
							dh_state[li * batch_n * hy_h * bi + (bacc + bs) * hy_h + h] += wei[bi * (in_h + hy_h) * hy_h + (numLayer - 1) * bi * (bi * hy_h + hy_h) * hy_h + w * hy_h + h] * dout[(bacc + bs) * out_h + w];
						}
					}
					else
					{
						// from doutput
						for (int w = 0; w < hy_h; w++)
						{
							dh_state[li * batch_n * hy_h * bi + (bacc + bs) * hy_h + h] += wei[bi * (in_h + hy_h) * hy_h + li * bi * (bi * hy_h + hy_h) * hy_h + h * hy_h + w] * dh_state[(li + 1) * batch_n * hy_h * bi + (bacc + bs) * hy_h + w];
							if (bidirection)
							{
								dh_state[li * batch_n * hy_h * bi + (bacc + bs) * hy_h + h] += wei[bi * (in_h + hy_h) * hy_h + li * bi * (bi * hy_h + hy_h) * hy_h + (bi * hy_h + hy_h) * hy_h + h * hy_h + w] * dh_state[(li + 1) * batch_n * hy_h * bi + batch_n * hy_h + (bacc + bs) * hy_h + w];
							}
						}
					}

					// from post state
					if (ti == seqLength - 1)
					{
						dh_state[li * batch_n * hy_h * bi + (bacc + bs) * hy_h + h] += dhy[li * bi * in_n[0] * hy_h + bs * hy_h + h];
					}
					else
					{
						dh_state[li * batch_n * hy_h * bi + (bacc + bs) * hy_h + h] += dhx_host[li * bi * in_n[0] * hy_h + bs * hy_h + h];
					}
					
					dh_state[li * batch_n * hy_h * bi + (bacc + bs) * hy_h + h] *= dervactivfunc(rsvspace[li * batch_n * hy_h * bi + (bacc + bs) * hy_h + h], squash);
					wkspace[li * batch_n * hy_h * bi + (bacc + bs) * hy_h + h] = dh_state[li * batch_n * hy_h * bi + (bacc + bs) * hy_h + h];
				}
					
				for (int h = 0; h < hy_h; h++)
				{
					dhx_host[li * bi * in_n[0] * hy_h + bs * hy_h + h] = 0;
					for (int w = 0; w < hy_h; w++)
					{
						if (li == 0)
						{
							dhx_host[li * bi * in_n[0] * hy_h + bs * hy_h + h] += wei[in_h * hy_h + h * hy_h + w] * dh_state[li * batch_n * hy_h * bi + (bacc + bs) * hy_h + w];
						}
						else
						{
							dhx_host[li * bi * in_n[0] * hy_h + bs * hy_h + h] += wei[bi * (in_h + hy_h) * hy_h + (li - 1) * bi * (bi * hy_h + hy_h) * hy_h + bi * hy_h * hy_h + h * hy_h + w] * dh_state[li * batch_n * hy_h * bi + (bacc + bs) * hy_h + w];
						}
					}
				}
			}
		}
		
		if (bidirection)
		{
			bacc = 0;
			for (int ti = 0; ti < seqLength; ti++)
			{
				for (int bs = 0; bs < in_n[ti]; bs++)
				{
					for (int h = 0; h < hy_h; h++)
					{
						if (li == numLayer - 1)
						{
							// from doutput
							for (int w = 0; w < out_h; w++)
							{
								dh_state[li * batch_n * hy_h * bi + batch_n * hy_h + (bacc + bs) * hy_h + h] += wei[bi * (in_h + hy_h) * hy_h + (numLayer - 1) * bi * (bi * hy_h + hy_h) * hy_h + out_h * hy_h + w * hy_h + h] * dout[(bacc + bs) * out_h + w];
							}
						}
						else
						{
							// from input
							for (int w = 0; w < hy_h; w++)
							{
								dh_state[li * batch_n * hy_h * bi + batch_n * hy_h + (bacc + bs) * hy_h + h] += wei[bi * (in_h + hy_h) * hy_h + li * bi * (bi * hy_h + hy_h) * hy_h + hy_h * hy_h + h * hy_h + w] * dh_state[(li + 1) * batch_n * hy_h * bi + (bacc + bs) * hy_h + w];
								if (bidirection)
								{
									dh_state[li * batch_n * hy_h * bi + batch_n * hy_h + (bacc + bs) * hy_h + h] += wei[bi * (in_h + hy_h) * hy_h + li * bi * (bi * hy_h + hy_h) * hy_h + (bi * hy_h + hy_h) * hy_h + hy_h * hy_h + h * hy_h + w] * dh_state[(li + 1) * batch_n * hy_h * bi + batch_n * hy_h + (bacc + bs) * hy_h + w];
								}
							}
						}

						// from post state
						if (ti == 0)
						{
							dh_state[li * batch_n * hy_h * bi + batch_n * hy_h + (bacc + bs) * hy_h + h] += dhy[li * bi * in_n[0] * hy_h + in_n[0] * hy_h + bs * hy_h + h];
						}
						else
						{
							dh_state[li * batch_n * hy_h * bi + batch_n * hy_h + (bacc + bs) * hy_h + h] += dhx_host[li * bi * in_n[0] * hy_h + in_n[0] * hy_h + bs * hy_h + h];
						}

						dh_state[li * batch_n * hy_h * bi + batch_n * hy_h + (bacc + bs) * hy_h + h] *= dervactivfunc(rsvspace[li * batch_n * hy_h * bi + batch_n * hy_h + (bacc + bs) * hy_h + h], squash);
						wkspace[li * batch_n * hy_h * bi + batch_n * hy_h + (bacc + bs) * hy_h + h] = dh_state[li * batch_n * hy_h * bi + batch_n * hy_h + (bacc + bs) * hy_h + h];
					}

					for (int h = 0; h < hy_h; h++)
					{
						dhx_host[li * bi * in_n[0] * hy_h + in_n[0] * hy_h + bs * hy_h + h] = 0;
						for (int w = 0; w < hy_h; w++)
						{
							if (li == 0)
							{
								dhx_host[li * bi * in_n[0] * hy_h + in_n[0] * hy_h + bs * hy_h + h] += wei[(in_h + hy_h) * hy_h + in_h * hy_h + h * hy_h + w] * dh_state[li * batch_n * hy_h * bi + batch_n * hy_h + (bacc + bs) * hy_h + w];
							}
							else
							{
								dhx_host[li * bi * in_n[0] * hy_h + in_n[0] * hy_h + bs * hy_h + h] += wei[bi * (in_h + hy_h) * hy_h + (li - 1) * bi * (bi * hy_h + hy_h) * hy_h + (bi * hy_h + hy_h) * hy_h + bi * hy_h * hy_h + h * hy_h + w] * dh_state[li * batch_n * hy_h * bi + batch_n * hy_h + (bacc + bs) * hy_h + w];
							}
						}
					}
				}
				bacc += in_n[ti];
			}
		}
	}

	// dinput
	bacc = 0;
	for (int ti = 0; ti < seqLength; ti++)
	{
		for (int bs = 0; bs < in_n[ti]; bs++)
		{
			for (int w = 0; w < in_h; w++)
			{
				for (int h = 0; h < hy_h; h++)
				{
					din_state[(bacc + bs) * in_h + w] += wei[w * hy_h + h] * dh_state[(bacc + bs) * hy_h + h];
					if (bidirection)
					{
						din_state[(bacc + bs) * in_h + w] += wei[(in_h + hy_h) * hy_h + w * hy_h + h] * dh_state[batch_n * hy_h + (bacc + bs) * hy_h + h];
					}
				}

				din_host[(bacc + bs) * in_h + w] = din_state[(bacc + bs) * in_h + w];
			}
		}
		bacc += in_n[ti];
	}
}


template <typename T>
void RunRNNBackwardWeightCPUVerify(std::vector<T>& in,
	std::vector<T>& dwei_host, // [ input_state_weight_trans  hidden_state_weight0_trans input1_trans hidden1_trans ... output_weight; bidirectional reversed weights ]
	std::vector<T>& hx, // initial hidden state
	std::vector<T>& dout,
	std::vector<int>& in_n, // input batch size
	int in_h, // input data length
	int seqLength, // Number of iterations to unroll over
	bool bidirection, // whether using bidirectional net
	int hy_d, // 1 by numlayer (number of stacks of hidden layers) for unidirection, 2 by numlayer for bidirection
	bool biased, // whether using bias
	int hy_n, // equal to input batch size in_n[0]
	int hy_h, // hidden state number
	std::vector<int>& out_n, // equals in_n
	int out_h;  // 1 by hy_h related function for unidirection, 2 by hy_h related function for bidirection
	std::vector<T>& rsvspace;
	std::vector<T>& wkspace
)
{
	int batch_n = sumvc(in_n);
	int numlayer = bidirection ? hy_d / 2 : hy_d;
	int out_dim = bidirection ? out_h / 2 : out_h;
	int bacc; // accumulation of batch
	int bi = bidirection ? 2 : 1;
	int squash = cudnnRNNMode_t == CUDNN_RNN_RELU ? 0 : 1;

	T * dwei_state = new T[(in_h + hy_h + out_h + (numlayer - 1) * (bi * hy_h + hy_h)) * bi * hy_h];
	memset(dwei_state, 0, (in_h + hy_h + out_h + (numlayer - 1) * (bi * hy_h + hy_h)) * bi * hy_h * sizeof(T));

	for (int li = 0; li <= numlayer; li++)
	{
		bacc = 0;
		for (int ti = 0; ti < seqLength; ti++)
		{
			if (li == 0)
			{
				// between layers
				for (int h = 0; h < in_h; h++)
				{
					for (int w = 0; w < hy_h; w++)
					{
						for (int bs = 0; bs < in_n[ti]; bs++)
						{
							dwei_state[h * hy_h + w] += in[(bacc + bs) * in_h + h] * wkspace[li * bi * batch_n * hy_h + (bacc + bs) * hy_h + w];
							if (bidirection)
							{
								dwei_state[(in_h + hy_h) * hy_h + h * hy_h + w] += in[(bacc + bs) * in_h + h] * wkspace[li * bi * batch_n * hy_h + batch_n * hy_h + (bacc + bs) * hy_h + w];
							}
						}
					}
				}

				// between time
				for (int h = 0; h < hy_h; h++)
				{
					for (int w = 0; w < hy_h; w++)
					{
						for (int bs = 0; bs < in_n[ti]; bs++)
						{
							if (ti == 0)
							{
								dwei_state[in_h * hy_h + (li - 1) * bi * (bi * hy_h + hy_h) * hy_h + bi * hy_h * hy_h + h * hy_h + w] += hx[li * bi * in_n[0] * hy_h + bs * hy_h + h] * wkspace[li * bi * batch_n * hy_h + (bacc + bs) * hy_h + w];
							}
							else
							{
								dwei_state[in_h * hy_h + (li - 1) * bi * (bi * hy_h + hy_h) * hy_h + bi * hy_h * hy_h + h * hy_h + w] += activfunc(rsvspace[li * bi * batch_n * hy_h + ((bacc - in_n[ti - 1]) + bs) * hy_h + h], squash) * wkspace[li * bi * batch_n * hy_h + (bacc + bs) * hy_h + w];
							}

							if (bidirection)
							{
								if (ti == seqLength - 1)
								{
									dwei_state[(in_h + hy_h) * hy_h + in_h * hy_h + (li - 1) * bi * (bi * hy_h + hy_h) * hy_h + (bi * hy_h + hy_h) * hy_h + bi * hy_h * hy_h + h * hy_h + w] += hx[li * bi * in_n[0] * hy_h + in_n[0] * hy_h + bs * hy_h + h] * wkspace[li * bi * batch_n * hy_h + batch_n * hy_h + (bacc + bs) * hy_h + w];
								}
								else
								{
									dwei_state[(in_h + hy_h) * hy_h + in_h * hy_h + (li - 1) * bi * (bi * hy_h + hy_h) * hy_h + (bi * hy_h + hy_h) * hy_h + bi * hy_h * hy_h + h * hy_h + w] += activfunc(rsvspace[li * bi * batch_n * hy_h + batch_n * hy_h + ((bacc + in_n[ti]) + bs) * hy_h + h], squash) * wkspace[li * bi * batch_n * hy_h + batch_n * hy_h + (bacc + bs) * hy_h + w];
								}
							}
						}

					}
				}
			}
			else if (li == numlayer)
			{
				// between layers
				for (int h = 0; h < out_h; h++)
				{
					for (int w = 0; w < hy_h; w++)
					{
						for (int bs = 0; bs < in_n[ti]; bs++)
						{
							dwei_state[bi * (in_h + hy_h) * hy_h + (li - 1) * bi * (bi * hy_h + hy_h) * hy_h + h * hy_h + w] += dout[(bacc + bs) * out_h + h] * activfunc(rsvspace[(li - 1) * bi * batch_n * hy_h + (bacc + bs) * hy_h + w], squash);
							if (bidirection)
							{
								dwei_state[bi * (in_h + hy_h) * hy_h + (li - 1) * bi * (bi * hy_h + hy_h) * hy_h + out_h * hy_h + h * hy_h + w] += dout[(bacc + bs) * out_h + h] * activfunc(rsvspace[(li - 1) * bi * batch_n * hy_h + batch_n * hy_h + (bacc + bs) * hy_h + w], squash);
							}
						}
					}
				}
			}
			else
			{
				// between layers
				for (int h = 0; h < hy_h; h++)
				{
					for (int w = 0; w < hy_h; w++)
					{
						for (int bs = 0; bs < in_n[ti]; bs++)
						{
							dwei_state[bi * (in_h + hy_h) * hy_h + (li - 1) * bi * (bi * hy_h + hy_h) * hy_h + h * hy_h + w] += activfunc(rsvspace[(li - 1) * bi * batch_n * hy_h + (bacc + bs) * hy_h + h], squash) * wkspace[li * bi * batch_n * hy_h + (bacc + bs) * hy_h + w];
							if (bidirection)
							{
								dwei_state[bi * (in_h + hy_h) * hy_h + (li - 1) * bi * (bi * hy_h + hy_h) * hy_h + hy_h * hy_h + h * hy_h + w] += activfunc(rsvspace[(li - 1) * bi * batch_n * hy_h + batch_n * hy_h + (bacc + bs) * hy_h + h], squash) * wkspace[li * bi * batch_n * hy_h + (bacc + bs) * hy_h + w];
								dwei_state[bi * (in_h + hy_h) * hy_h + (li - 1) * bi * (bi * hy_h + hy_h) * hy_h + (bi * hy_h + hy_h) * hy_h + h * hy_h + w] += activfunc(rsvspace[(li - 1) * bi * batch_n * hy_h + (bacc + bs) * hy_h + h], squash) * wkspace[li * bi * batch_n * hy_h + batch_n * hy_h + (bacc + bs) * hy_h + w];
								dwei_state[bi * (in_h + hy_h) * hy_h + (li - 1) * bi * (bi * hy_h + hy_h) * hy_h + (bi * hy_h + hy_h) * hy_h + hy_h * hy_h + h * hy_h + w] += activfunc(rsvspace[(li - 1) * bi * batch_n * hy_h + batch_n * hy_h + (bacc + bs) * hy_h + h], squash) * wkspace[li * bi * batch_n * hy_h + batch_n * hy_h + (bacc + bs) * hy_h + w];
							}
						}
					}
				}			

				// between time
				for (int h = 0; h < hy_h; h++)
				{
					for (int w = 0; w < hy_h; w++)
					{
						for (int bs = 0; bs < in_n[ti]; bs++)
						{
							if (ti == 0)
							{
								dwei_state[bi * (in_h + hy_h) * hy_h + (li - 1) * bi * (bi * hy_h + hy_h) * hy_h + bi * hy_h * hy_h + h * hy_h + w] += hx[li * bi * in_n[0] * hy_h + bs * hy_h + h] * wkspace[li * bi * batch_n * hy_h + (bacc + bs) * hy_h + w];
							}
							else
							{
								dwei_state[bi * (in_h + hy_h) * hy_h + (li - 1) * bi * (bi * hy_h + hy_h) * hy_h + bi * hy_h * hy_h + h * hy_h + w] += activfunc(rsvspace[li * bi * batch_n * hy_h + ((bacc - in_n[ti - 1]) + bs) * hy_h + h], squash) * wkspace[li * bi * batch_n * hy_h + (bacc + bs) * hy_h + w];
							}

							if (bidirection)
							{
								if (ti == seqLength - 1)
								{
									dwei_state[bi * (in_h + hy_h) * hy_h + (li - 1) * bi * (bi * hy_h + hy_h) * hy_h + (bi * hy_h + hy_h) * hy_h + bi * hy_h * hy_h + h * hy_h + w] += hx[li * bi * in_n[0] * hy_h + in_n[0] * hy_h + bs * hy_h + h] * wkspace[li * bi * batch_n * hy_h + batch_n * hy_h + (bacc + bs) * hy_h + w];
								}
								else
								{
									dwei_state[bi * (in_h + hy_h) * hy_h + (li - 1) * bi * (bi * hy_h + hy_h) * hy_h + (bi * hy_h + hy_h) * hy_h + bi * hy_h * hy_h + h * hy_h + w] += activfunc(rsvspace[li * bi * batch_n * hy_h + batch_n * hy_h + ((bacc + in_n[ti]) + bs) * hy_h + h], squash) * wkspace[li * bi * batch_n * hy_h + batch_n * hy_h + (bacc + bs) * hy_h + w];
								}
							}
						}

					}
				}
			}
			
			//for bias
			if (biased)
			{
				if (li == 0)
				{
					for (int h = 0; h < hy_h; h++)
					{
						for (int w = 0; w < batch_n; w++)
						{
							dwei_state[((in_h + hy_h + out_h) * bi + (bi * hy_h + hy_h) * bi * (numLayer - 1)) * hy_h + h] += rsvspace[li * bi * batch_n * hy_h + w* hy_h + h];

							if (bidirection)
							{
								dwei_state[((in_h + hy_h + out_h) * bi + (bi * hy_h + hy_h) * bi * (numLayer - 1)) * hy_h + 2 * hy_h + h] += rsvspace[li * bi * batch_n * hy_h + batch_n * hy_h + w* hy_h + h];
							}
						}

						dwei_state[((in_h + hy_h + out_h) * bi + (bi * hy_h + hy_h) * bi * (numLayer - 1)) * hy_h + hy_h + h] = dwei_state[((in_h + hy_h + out_h) * bi + (bi * hy_h + hy_h) * bi * (numLayer - 1)) * hy_h + h];
						
						if (bidirection)
						{
							dwei_state[((in_h + hy_h + out_h) * bi + (bi * hy_h + hy_h) * bi * (numLayer - 1)) * hy_h + 3 * hy_h + h] = dwei_state[((in_h + hy_h + out_h) * bi + (bi * hy_h + hy_h) * bi * (numLayer - 1)) * hy_h + 2 * hy_h + h];
						}
					}
				}
				else if (li == numlayer)
				{
					for (int h = 0; h < out_h; h++)
					{
						for (int w = 0; w < batch_n; w++)
						{
							dwei_state[((in_h + hy_h + out_h) * bi + (bi * hy_h + hy_h) * bi * (numLayer - 1)) * hy_h + (bi * 2 + bi * (bi + 1) * (numLayer - 1)) * hy_h + h] += dout[w * hy_h + h];

							if (bidirection)
							{
								dwei_state[((in_h + hy_h + out_h) * bi + (bi * hy_h + hy_h) * bi * (numLayer - 1)) * hy_h + (bi * 2 + bi * (bi + 1) * (numLayer - 1)) * hy_h + out_h + h] = dwei_state[((in_h + hy_h + out_h) * bi + (bi * hy_h + hy_h) * bi * (numLayer - 1)) * hy_h + (bi * 2 + bi * (bi + 1) * (numLayer - 1)) * hy_h + h];
							}
						}
					}
				}
				else
				{
					for (int h = 0; h < hy_h; h++)
					{
						for (int w = 0; w < batch_n; w++)
						{
							dwei_state[((in_h + hy_h + out_h) * bi + (bi * hy_h + hy_h) * bi * (numLayer - 1)) * hy_h + bi * 2 * hy_h + (li - 1) * bi * (bi + 1) * hy_h + h] += rsvspace[li * bi * batch_n * hy_h + w* hy_h + h];

							if (bidirection)
							{
								dwei_state[((in_h + hy_h + out_h) * bi + (bi * hy_h + hy_h) * bi * (numLayer - 1)) * hy_h + bi * 2 * hy_h + (li - 1) * bi * (bi + 1) * hy_h + (bi + 1) * hy_h + h] += rsvspace[li * bi * batch_n * hy_h + batch_n * hy_h + w* hy_h + h];
							}
						}

						dwei_state[((in_h + hy_h + out_h) * bi + (bi * hy_h + hy_h) * bi * (numLayer - 1)) * hy_h + bi * 2 * hy_h + (li - 1) * bi * (bi + 1) * hy_h + bi * hy_h + h] = dwei_state[((in_h + hy_h + out_h) * bi + (bi * hy_h + hy_h) * bi * (numLayer - 1)) * hy_h + bi * 2 * hy_h + (li - 1) * bi * (bi + 1) * hy_h + h];
						
						if (bidirection)
						{
							dwei_state[((in_h + hy_h + out_h) * bi + (bi * hy_h + hy_h) * bi * (numLayer - 1)) * hy_h + bi * 2 * hy_h + (li - 1) * bi * (bi + 1) * hy_h + hy_h + h] = dwei_state[((in_h + hy_h + out_h) * bi + (bi * hy_h + hy_h) * bi * (numLayer - 1)) * hy_h + bi * 2 * hy_h + (li - 1) * bi * (bi + 1) * hy_h + h];

							dwei_state[((in_h + hy_h + out_h) * bi + (bi * hy_h + hy_h) * bi * (numLayer - 1)) * hy_h + bi * 2 * hy_h + (li - 1) * bi * (bi + 1) * hy_h + (bi + 1) * hy_h + hy_h + h] = dwei_state[((in_h + hy_h + out_h) * bi + (bi * hy_h + hy_h) * bi * (numLayer - 1)) * hy_h + bi * 2 * hy_h + (li - 1) * bi * (bi + 1) * hy_h + (bi + 1) * hy_h + h];
							dwei_state[((in_h + hy_h + out_h) * bi + (bi * hy_h + hy_h) * bi * (numLayer - 1)) * hy_h + bi * 2 * hy_h + (li - 1) * bi * (bi + 1) * hy_h + (bi + 1) * hy_h + bi * hy_h + h] = dwei_state[((in_h + hy_h + out_h) * bi + (bi * hy_h + hy_h) * bi * (numLayer - 1)) * hy_h + bi * 2 * hy_h + (li - 1) * bi * (bi + 1) * hy_h + (bi + 1) * hy_h + h];
						}
					}
				}
			}

			bacc += in_n[ti];
		}
	}

	for (int i = 0; i < (in_h + hy_h + out_h + (numlayer - 1) * (bi * hy_h + hy_h)) * bi * hy_h; i++)
	{
		dwei_host[i] = dwei_state[i];
	}
}