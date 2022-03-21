/*
 * odesolver.h
 *
 *  Created on: Nov 17, 2021
 *      Author: vinic
 */

#ifndef ODESOLVER_H_
#define ODESOLVER_H_

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
#pragma once

#include <stdlib.h>
#include <stdbool.h>

// O usuário pode definir o número máximo de equações diferenciais no sistema.
// Para esse trabalho, o máximo será 2 do modelo de Izhikevich, mas se for
// necessário, é possível aumentar.
#ifndef MAX_EQUATIONS
#define MAX_EQUATIONS 8
#endif

#ifndef ODE_TYPE
#define ODE_TYPE float
#endif
/**
 * @brief Define o tipo de dado a ser utilizado para o cálculo das equações
 * diferenciais. No caso é utilizado um float.
 */
typedef ODE_TYPE ode_data_type;

/**
 * @brief Enumeração com o status das operações sobre o solver.
 *
 */
typedef enum ode_status
{
    /**
     * @brief A operação ocorreu normalmente.
     *
     */
    ODE_OK,
    /**
     * @brief Houve uma falha de alocação de memória durante a operação.
     *
     */
    ODE_OUT_OF_MEMORY,

    ODE_INVALID_PARAMETER
} ode_status_t;

/**
 * @brief Esse é um ponteiro de função que define o modelo através das equações
 * diferenciais F(t, y).
 *
 * Esse prótipo é baseado no solver do GLS
 * https://www.gnu.org/software/gsl/doc/html/ode-initval.html.
 * e a solução seria calculada numericamente pelo ODE.
 *
 * Por questões de gerenciamento de memória, a função por padrão é limitada a
 * sistemas com até 8 equações diferenciais. Caso deseje utilizar sistemas com
 * mais de 8 equações, defina o valor máximo usando a macro MAX_EQUATIONS.
 * Ex.
 * #define MAX_EQUATIONS 12
 *
 * Parâmetros:
 * t - O tempo atual, a ser usadado na equação diferencial.
 * y[] seriam os estados iniciais/anteriores.
 * f[] seriam os novos estados calculados (dy/dt).
 * params seriam outros parâmetros internos. Por exemplo as constantes do modelo
 * neural (a,b,c e d). Também pode ser utilizado para incluir entradas extras no
 * solver.
 *
 */
typedef ode_status_t (*diff_equation_model)(const ode_data_type t,
                                            const ode_data_type *const y,
                                            ode_data_type *const df,
                                            void *const params);

/**
 * @brief Estrutura que mantém um modelo de um sistema dinâmico usando equações
 * diferenciais ordinárias.
 *
 * Por questões de gerenciamento de memória, a função por padrão é limitada a
 * sistemas com até 8 equações diferenciais. Caso dedese utilizar sistemas com
 * mais de 8 equações, defina o valor máximo usando a macro MAX_EQUATIONS.
 * Ex.
 * #define MAX_EQUATIONS 12
 *
 */
typedef struct ode_system
{
    /**
     * @brief Ponteiro para função com o modelo. Essa função calcula a derivada
     * da equação diferencial.
    */
    diff_equation_model model_function;
    /**
     * @brief Número de equações na função do modelo. O vetor y tem o mesmo
     * número de equações.
     */
    size_t num_equations;
    /**
     * @brief  Parâmetros específicos passado para a função do modelo.
     */
    void *model_params;
    /**
     * @brief Estado atual. É atualizado ao executar um passo da função.
     */
    ode_data_type *y;
    /**
     * @brief Tempo atual. É atualizado ao executar um passo da função.
     *
     */
    ode_data_type t;
} t_ode_system;

/**
 * @brief Executa um passo do método de Euler (ODE1). Esse método resolve uma
 * equação diferencial numericamente.
 *
 * Ver detalhes do método de Euler em
 * https://pt.wikipedia.org/wiki/M%C3%A9todo_de_Euler.
 *
 * Por questões de gerenciamento de memória, a função por padrão é limitada a
 * sistemas com até 8 equações diferenciais. Caso dedese utilizar sistemas com
 * mais de 8 equações, defina o valor máximo usando a macro MAX_EQUATIONS.
 * Ex.
 * #define MAX_EQUATIONS 12
 *
 * @param model O modelo armazenado em uma equação diferencial. Ver detalhes na
 * estrutura ode_system.
 * @param h O intervalo do passo (dt) para o calculo da próxima iteração.
 * @return int O status da execução do passo.
 */
ode_status_t ode1_step(t_ode_system *const model, const ode_data_type h);

/********* Alpha Function *****************************************************/

/**
 * @brief Função que apresenta um modelo do potencial pós-sinaptico usando um
 * decaimento exponencial. Esse decaimento é conhecido como Alpha Function.
 *
 * @param t O tempo atual.
 * @param y Os valores de entrada para calcular o decaimento exponencial. Nesse
 * caso é experando um ponteiro para um único valor do tipo ode_data_type.
 * @param df Um ponteiro onde é calculado o resultado do decaimento exponencial.
 * @param params Ponteiro para os parâmetros da função. Esses parâmetros são do
 * tipo {@link #alpha_function_params alpha_function_params}.
 * @return ode_status_t O status da execução da função.
 */
ode_status_t alpha_function_model(const ode_data_type t,
                                  const ode_data_type *const y,
                                  ode_data_type *const df,
                                  void *const params);

/**
 * @brief Cria um modelo dinâmico do potencial pós-sináptico baseado na alpha
 * function.
 *
 * Essa função aloca dinâmicamente o modelo e parâmetros. Ao terminar de usar o
 * modelo deve-se liberar a memória usando a função
 * @link{#alpha_function_free(t_ode_system * system) alpha_function_free}
 * .
 *
 * @param model Ponteiro para o vetor onde será armazenado o modelo para
 * utilização com o solucionador (solver) das equações diferenciais.
 * @param tau O tempo de vida médio do decaimento exponencial.
 * @param weight O peso a ser aplicado sobre o sinal.
 * @return ode_status_t O status da execução.
 */
ode_status_t alpha_function_alloc(t_ode_system **model,
                                  const ode_data_type tau,
                                  const ode_data_type weight);

/**
 * @brief Libera a memória de um modelo alpha function
 *
 * @param system Ponteiro para o systema cuja memória será liberada.
 * @return ode_status_t
 */
ode_status_t alpha_function_free(t_ode_system *system);

/**
 * @brief Estrutura que define os parâmetros do modelo de potencial
 * pós-sináptico usando a alpha function.
 *
 *
 */
typedef struct alpha_function_params
{
    /**
     * @brief Thau - corresponde ao tempo de vida médio do decaimento
     * exponencial. Aqui armazena o valor ineverso para evitar uma divisão
     * (one_over_thau).
     *
     */
    ode_data_type one_over_thau;

    /**
     * @brief weight - É um peso que é aplicado à entrada da função. Ele pode
     * ser utilizado para definir diferentes pesos para diferentes conexões.
     * Se não desejar usar esse recurso, o peso deve ser definido como sendo 1.
     */
    ode_data_type weight;

    /**
     * @brief  I - É a corrente de entrada aplicada ao modelo. Geralmente
     * corresponde a um conjunto de spikes, podendo receber 0 ou 1.
     *
     */
    ode_data_type I;
} t_alpha_function_params;

/**
 * @brief Compara dois objetos com modelo de Alpha Function. Retorna verdadeiro
 * se eles são iguais, ou seja, têm os mesmos parâmetros.
 *
 * @param a O primeiro sistema a ser comparado
 * @param b O segundo sistema a ser comparado
 * @return true Os sistemas a e b são iguais
 * @return false Os sistemas a e b são diferentes
 */
bool is_alpha_model_equal(t_ode_system *a, t_ode_system *b);

/********* Izhikevich *********************************************************/

/**
 * @brief Cria um modelo dinâmico de um neurônio segundo a proposta de
 * Izhikevich.
 *
 * Ver "Izhikevich, E. M. Simple Model of Spiking Neurons. IEEE TRANSACTIONS ON
 * NEURAL NETWORKS, VOL. 14, NO. 6, NOVEMBER 2003" para maiores detalhes sobre
 * esse modelo de neurônio.
 *
 * Essa função aloca dinâmicamente o modelo e parâmetros. Ao terminar de usar o
 * modelo deve-se liberar a memória usando a função
 * @link{#izhikevich_free(t_ode_system * system) izhikevich_free}.
 *
 * @param model O ponteiro para o modelo a ser criado.
 * @param a O parâmetro a do modelo de Izhikevich.
 * @param b O parâmetro b do modelo de Izhikevich.
 * @param c O parâmetro c do modelo de Izhikevich.
 * @param d O parâmetro d do modelo de Izhikevich.
 * @return ode_status_t O status da execução da função.
 */
ode_status_t izhikevich_alloc(t_ode_system **model,
                              const ode_data_type a,
                              const ode_data_type b,
                              const ode_data_type c,
                              const ode_data_type d);

/**
 * @brief Compara dois objetos com modelo de Izhikevich. Retorna verdadeiro se
 * eles são iguais, ou seja, têm os mesmos parâmetros.
 *
 * @param a O primeiro sistema a ser comparado
 * @param b O segundo sistema a ser comparado
 * @return true Os sistemas a e b são iguais
 * @return false Os sistemas a e b são diferentes
 */
bool is_izhikevich_model_equal(t_ode_system *a, t_ode_system *b);

/**
 * @brief Função que implementa as equações diferenciais do Modelo de
 * Izhikevich. Essa função se limita as equações diferenciais, com o passo
 * padrão de 1 ms e as tensões em mV.
 *
 * IMPORTANTE: Essa função só implementa o modelo de equações diferenciais. O
 * reset do modelo é implementado a parte na função izhikevich_step que além de
 * realizar o passo pelo solver, avalia o reset.
 *
 * Ver "Izhikevich, E. M. Simple Model of Spiking Neurons. IEEE TRANSACTIONS ON
 * NEURAL NETWORKS, VOL. 14, NO. 6, NOVEMBER 2003" para maiores detalhes.
 *
 * @param t Tempo. Não é usado no modelo, mas é requerido pelo solver.
 * @param y As entradas do modelo. É esperado um vetor com duas posições do tipo
 * ode_data_type.
 * @param df As respostas das equações diferenciais a serem passadas para o
 * solver. Deve-se passar um vetor com duas posições do tipo ode_data_type.
 * @param params Os parâmetros do modelo. Deve-se passar um ponteiro para uma
 * estrutura do tipo izhikevich_params.
 * @return ode_status_t O status da execução da equação do modelo.
 */
ode_status_t izhikevich_model(const ode_data_type t,
                              const ode_data_type *const y,
                              ode_data_type *const df,
                              void *const params);

/**
 * @brief Libera a memória alocada para um neurônio de Izhikevich.
 *
 * @param system O ponteiro para o modelo de neurônio a ser removido da memória.
 */
void izhikevich_free(t_ode_system *system);

/**
 * @brief Executa um passo do modelo de Izhikevich.
 *
 * Essa função extende a funcionalidade do ode_step, adicionado o reset
 * específico do modelo de Izhikevich. Esse reset é uma não-linearidade do
 * modelo que reseta os potenciais internos ao ultrapassar o limiar. Essa função
 * foi criada para evitar que as entradadas do solver fossem modificadas dentro
 * dele. Para isso, o reset é criado a parte. Isso garante que as entradas são
 * do tipo const. Fora esse detalhe específico, essa função opera de forma
 * equivalente ao ode_step.
 *
 * @param model O sistema que contém o modelo de Izhikevich.
 * @param h O intervalo de tempo do passo.
 * @return ode_status_t O status da execução do passo.
 */
ode_status_t izhikevich_step(t_ode_system *const model, const ode_data_type h);

/**
 * @brief Estrutura que mantém os parâmetros do modelo de Izhikevich
 *
 */
typedef struct izhikevich_params
{
    /**
     * @brief O parâmetro a do modelo de Izhikevich.
     *
     */
    ode_data_type a;

    /**
     * @brief O parâmetro b do modelo de Izhikevich.
     *
     */
    ode_data_type b;

    /**
     * @brief O parâmetro c do modelo de Izhikevich.
     *
     */
    ode_data_type c;

    /**
     * @brief O parâmetro d do modelo de Izhikevich.
     *
     */
    ode_data_type d;

    /**
     * @brief A corrente de entrada a ser utilizada no modelo de Izhichevich.
     *
     */
    ode_data_type I;

    /**
     * @brief Esse parâmetro é atualizado pela função do modelo. Ele indica se o
     * modelo gerou um spike. Recebe apenas valores 0 e 1, gerando um sinal
     * binário.
     *
     */
    int spike;
} t_izhikevich_params;

/********* Izhikevich  e pontencial pós-sinaptico combinados ******************/

/**
 * @brief Processa um neurônio de Izhikevich e o seu potencial pós-sinápitco
 * combinados.
 *
 * @param izhikevich_neuron O sistema com o modelo de Izhikevich.
 * @param alphafunction O sistema com o modelo do potencial pós-sináptico.
 * @param input_current A corrente de entrada
 * @param step O intervalo de tempo do passo.
 * @return ode_data_type O status da execução do passo.
 */
ode_data_type process_neuron_step(t_ode_system *izhikevich_neuron,
                                  t_ode_system *alphafunction,
                                  ode_data_type input_current,
                                  ode_data_type step);

/**
 * @brief Processa um conjunto de neurônios de Izhikevich e combina o potencial
 * pós-sinápitico de todos.
 *
 * @param izhikevich_neuron Um array com N sistemas que modelam neurônios de
 * Izhikevich. Os nurônios devem ser individuais e não podem ser reutilizados.
 * @param alphafunction Um array com N sistemas que modelam o potencial pós-
 * sinápitco. Os sistemas devem ser individuais e não podem ser reutilizados.
 * @param input_currents Um array com as correntes de entrada de cada neurônio.
 * @param n_neurons O número de neurônios (N)
 * @param step O tempo a ser avaliado no passo.
 * @return ode_data_type O resultado da avaliação de todos os neurônios.
 */
ode_data_type process_neuron_set_step(t_ode_system **izhikevich_neuron,
                                      t_ode_system **alphafunction,
                                      ode_data_type *input_currents,
                                      int n_neurons,
                                      ode_data_type step);

/**
 * @brief Cria o modelo de um conjunto (rede) de neurônios combinados.
 *
 * @param izhikevich_neurons Um array de ponteiros para modelos de Izhikevich.
 * Esse array deve possuir N elementos.
 * @param alphafunctions Um array de ponteiros para modelos de potencial pós-
 * sináptico (alpha function). Esse array deve possuir N elementos.
 * @param num_neurons  O número de neurônios da rede (N).
 * @param a O parâmetro a do modelo de Izhikevich.
 * @param b O parâmetro b do modelo de Izhikevich.
 * @param c O parâmetro c do modelo de Izhikevich.
 * @param d O parâmetro d do modelo de Izhikevich.
 * @param tau O parâmetro tau do decaimento exponencial (Potencial pós-
 * sináptico)
 * @param weights Um array de pesos de tamanho N a serem aplicados a cada
 * potencial pós-sináptico.
 * @return ode_status_t O status da criação do conjunto de neurônio.
 */
ode_status_t fill_neuron_set(t_ode_system *izhikevich_neurons[],
                             t_ode_system *alphafunctions[],
                             int num_neurons,
                             ode_data_type a, ode_data_type b,
                             ode_data_type c, ode_data_type d,
                             ode_data_type tau,
                             ode_data_type *weights);

/* [] END OF FILE */

#endif /* ODESOLVER_H_ */
