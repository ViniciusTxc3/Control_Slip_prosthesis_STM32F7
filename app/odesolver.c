/*
 * odesolver.c
 *
 *  Created on: Nov 17, 2021
 *      Author: vinic
 */

/* ========================================
 *
 * Copyright Ailton Luiz Dias Siqueira Junior, 2021
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF Ailton Luiz Dias Siqueira Junior.
 *
 * ========================================
*/
#include "odesolver.h"
#include <stdlib.h>
#include <string.h>

// Macros de otimização. Elas são usadas para melhorar a questão da previsão de
// branch. Ver https://kernelnewbies.org/FAQ/LikelyUnlikely
#define likely(x) __builtin_expect(!!(x), 1)
#define unlikely(x) __builtin_expect(!!(x), 0)

_Bool model_parameters_is_valid(const ode_data_type *const y,
                                ode_data_type *const df,
                                void *const params)
{
    return (y == NULL) | (df == NULL) | (params == NULL);
}

ode_status_t ode1_step(t_ode_system *const model, const ode_data_type h)
{

    // Verificação das entradas, pode ser desabilitada definindo a macro
    // ODE_UNSAFE_MODE para melhor desempenho
#ifndef ODE_UNSAFE_MODE
    if (unlikely(model == NULL))
        return ODE_INVALID_PARAMETER;
#endif

    // Tive problemas com o malloc antes com o PSoC (Veja o comentário abaixo).
    // Ele foi resolvido aumentando o tamanho do Heap que é muito pequeno para
    // esses dispositivos. Ainda assim optei por uma alocação estática aqui por
    // questões de velocidade. Por padrão acaba alocando espaço para 8 equações.
    // Caso isso não seja suficiente o usuário pode definir um número maior de
    // equações.
    //
    // Não sei por que o malloc não funciona. Ao que parece a função é bastante
    // limitada em um sistema embarcado. Com isso estou definindo um tamanho
    // fixo para dy, o que limita o código a no máximo 8 equações.
    // ode_data_type * dy =
    //   (ode_data_type *) malloc(model->num_equations * sizeof(ode_data_type));
    // if(dy == NULL) return ODE_OUT_OF_MEMORY;
    ode_data_type dy[MAX_EQUATIONS];

    //typedef int (*diffFunction)(
    //           ode_data_type t, const ode_data_type y[], ode_data_type df[],
    //           void *params);  // Declare typedef
    model->model_function(model->t, model->y, dy, model->model_params);

    for (unsigned int i = 0; i < model->num_equations; i++)
    {
        model->y[i] = model->y[i] + h * dy[i];
    }

    model->t = model->t + h;
    //free(dy);

    return ODE_OK;
}

bool is_base_model_equal(t_ode_system *a, t_ode_system *b)
{
    if (a == b)
        return true;
    if (a->model_function != b->model_function)
        return false;
    if (a->num_equations != b->num_equations)
        return false;
    if (memcmp(a->y, b->y, a->num_equations * sizeof(ode_data_type)))
        return false;
    if (a->t != b->t)
        return false;
    return true;
}

/********* Alpha Function *****************************************************/

ode_status_t alpha_function_model(const ode_data_type t,
                                  const ode_data_type *const y,
                                  ode_data_type *const df,
                                  void *const params)
{
    // Verificação das entradas, pode ser desabilitada definindo a macro
    // ODE_UNSAFE_MODE para melhor desempenho
#ifndef ODE_UNSAFE_MODE
    if (unlikely(model_parameters_is_valid(y, df, params)))
        return ODE_INVALID_PARAMETER;
#endif

    t_alpha_function_params *alpha_params =
        (t_alpha_function_params *)params;
    //v' = -1/tau * v + (I*weight)
    df[0] = alpha_params->one_over_thau * y[0] +
            (alpha_params->I * alpha_params->weight);

    return ODE_OK;
}

ode_status_t alpha_function_alloc(t_ode_system **model, const ode_data_type tau,
                                  const ode_data_type weight)
{

    t_ode_system *simple_system = malloc(sizeof(t_ode_system));

    if (simple_system == NULL)
        return ODE_OUT_OF_MEMORY;

    t_alpha_function_params *params = malloc(sizeof(t_alpha_function_params));

    if (params == NULL)
        return ODE_OUT_OF_MEMORY;

    ode_data_type *initial_state = malloc(sizeof(ode_data_type));
    initial_state[0] = 0;

    if (initial_state == NULL)
        return ODE_OUT_OF_MEMORY;

    params->one_over_thau = -1.0 / tau;
    params->weight = weight;
    params->I = 0;

    simple_system->model_function = &alpha_function_model;
    simple_system->model_params = params;
    simple_system->num_equations = 1;
    simple_system->t = 0;
    simple_system->y = initial_state;

    *model = simple_system;

    return ODE_OK;
}

ode_status_t alpha_function_free(t_ode_system *system)
{
    if ((system != NULL))
    {
        free(system->y);
        free(system->model_params);
        free(system);
    }
    return ODE_OK;
}

bool is_alpha_model_equal(t_ode_system *a, t_ode_system *b)
{
    if (!is_base_model_equal(a, b))
        return false;

    //Compara os parâmetros
    t_alpha_function_params *a_params =
        (t_alpha_function_params *)a->model_params;
    t_alpha_function_params *b_params =
        (t_alpha_function_params *)b->model_params;

    if (memcmp(a_params, b_params, sizeof(t_alpha_function_params)))
        return false;

    return true;
}

/********* Izhikevich *********************************************************/

// Remova o comentário abaixo para desativar o limite inferior de -100mV no
// modelo
// #define DISABLE_LOWER_LIMIT_IZICHEVICH

ode_status_t izhikevich_model(const ode_data_type t,
                              const ode_data_type *const y,
                              ode_data_type *const df,
                              void *const params)
{

    // Verificação das entradas, pode ser desabilitada definindo a macro
    // ODE_UNSAFE_MODE para melhor desempenho
#ifndef ODE_UNSAFE_MODE
    if (unlikely(model_parameters_is_valid(y, df, params)))
        return ODE_INVALID_PARAMETER;
#endif

    // a, b, c, d e I
    t_izhikevich_params *izhikevich_params = (t_izhikevich_params *)params;

    //v' -> df[0]
    //v -> y[0]
    //u' -> df[1]
    //u -> y[1]
    ode_data_type I_in = (izhikevich_params->I);
    //v' = 0.04v^2 + 5v + 140 -u + I
    df[0] = 0.04 * y[0] * y[0] + 5.0 * y[0] + 140.0 - y[1] + I_in;

    //u' = a(bv - u)
    df[1] = izhikevich_params->a * (izhikevich_params->b * y[0] - y[1]);

    return ODE_OK;
}

ode_status_t izhikevich_alloc(t_ode_system **model,
                              const ode_data_type a,
                              const ode_data_type b,
                              const ode_data_type c,
                              const ode_data_type d)
{
    t_ode_system *izhichevich_neuron = malloc(sizeof(t_ode_system));

    if (izhichevich_neuron == NULL)
        return ODE_OUT_OF_MEMORY;

    ode_data_type *initial_state = malloc(2 * sizeof(ode_data_type));

    if (initial_state == NULL)
        return ODE_OUT_OF_MEMORY;

    initial_state[0] = c;
    initial_state[1] = -d;

    t_izhikevich_params *model_params = malloc(sizeof(t_izhikevich_params));

    if (model_params == NULL)
        return ODE_OUT_OF_MEMORY;

    model_params->a = a;
    model_params->b = b;
    model_params->c = c;
    model_params->d = d;
    model_params->I = 0;
    model_params->spike = 0;

    izhichevich_neuron->model_function = &izhikevich_model;

    izhichevich_neuron->model_params = model_params;
    izhichevich_neuron->num_equations = 2;
    izhichevich_neuron->t = 0;
    izhichevich_neuron->y = initial_state;

    *model = izhichevich_neuron;

    return ODE_OK;
}

bool is_izhikevich_model_equal(t_ode_system *a, t_ode_system *b)
{

    if (!is_base_model_equal(a, b))
        return false;

    t_izhikevich_params *a_params = (t_izhikevich_params *)a->model_params;
    t_izhikevich_params *b_params = (t_izhikevich_params *)b->model_params;

    if (memcmp(a_params, b_params, sizeof(t_izhikevich_params)))
        return false;

    return true;
}

void izhikevich_free(t_ode_system *system)
{
    if ((system != NULL))
    {
        free(system->y);
        free(system->model_params);
        free(system);
    }
}

ode_status_t izhikevich_step(t_ode_system *const model, const ode_data_type h)
{

    t_izhikevich_params *model_params =
        (t_izhikevich_params *)model->model_params;
    //Reset
    if (unlikely(model->y[0] >= 30.0))
    {
        model->y[0] = model_params->c;
        model->y[1] = model->y[1] + model_params->d;
        model_params->spike = 1;
    }
    else
    {
        model_params->spike = 0;
    }

    int status = ode1_step(model, h);

    if (unlikely(model->y[0] >= 35))
    {
        model->y[0] = 35;
    }

    // Adicionei esse limite no valor mínimo pois o modelo apresentava um
    // comportamento estranho, semelhante a um bug. Em alguns momentos, essa
    // tensão ficava muito negativa, e quando era elevada ao quadrado, o modelo
    // entrava em um estado que fica travado, oscilando. É preciso investigar
    // melhor o que está acontecendo.
#ifndef DISABLE_LOWER_LIMIT_IZICHEVICH
    if (unlikely(model->y[0] <= -100.0))
    {
        model->y[0] = -100.0;
    }
#endif

    return status;
}

/********* Izhikevich  e pontencial pós-sinaptico combinados ******************/
/********* Alpha Function (Potencial pós-sinaptico) ***************************/

ode_data_type process_neuron_step(t_ode_system *izhikevich_neuron,
                                  t_ode_system *alphafunction,
                                  ode_data_type input_current,
                                  ode_data_type step)
{
    t_izhikevich_params *izichevich_params =
        (t_izhikevich_params *)izhikevich_neuron->model_params;

    izichevich_params->I = input_current;

    izhikevich_step(izhikevich_neuron, step);

    t_alpha_function_params *alpha_params =
        (t_alpha_function_params *)alphafunction->model_params;

    alpha_params->I = izichevich_params->spike;

    ode1_step(alphafunction, 0.001);

    return alphafunction->y[0];
}

ode_data_type process_neuron_set_step(t_ode_system **izhikevich_neuron,
                                      t_ode_system **alphafunction,
                                      ode_data_type *input_currents,
                                      int n_neurons,
                                      ode_data_type step)
{
    ode_data_type response = 0;

    for (int neuron_index = 0; neuron_index < n_neurons; neuron_index++)
    {
        response += process_neuron_step(izhikevich_neuron[neuron_index],
                                        alphafunction[neuron_index],
                                        input_currents[neuron_index], step);
    }

    return response;
}

ode_status_t fill_neuron_set(t_ode_system *izhikevich_neurons[],
                             t_ode_system *alphafunctions[],
                             int num_neurons,
                             ode_data_type a, ode_data_type b,
                             ode_data_type c, ode_data_type d,
                             ode_data_type tau,
                             ode_data_type *weights)
{

    for (int neuron_index = 0; neuron_index < num_neurons; neuron_index++)
    {

        izhikevich_alloc(&(izhikevich_neurons[neuron_index]), a, b, c, d);
        if (izhikevich_neurons[neuron_index] == NULL)
            return ODE_OUT_OF_MEMORY;

        alpha_function_alloc(&(alphafunctions[neuron_index]),
            tau, weights[neuron_index]);
        if (alphafunctions[neuron_index] == NULL)
            return ODE_OUT_OF_MEMORY;
    }

    return ODE_OK;
}

/* [] END OF FILE */
