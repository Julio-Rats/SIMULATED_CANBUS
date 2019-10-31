#ifndef  SIMULATOR
#define   SIMULATOR

#include <cmath>
#include <vector>
#include <iostream>

#define     DURATION_FRAME       ((double)(pow(2,20)/1000))
#define     ERRO_INPUT_CANDB     13
#define     k_chebychev          5
/******************************************************************

Estrutura (classe) para representar um frame CAN

*******************************************************************/

class Frame_CAN
{
  public:
      unsigned int  id:11;               // ID da mensagem
      double        cycle_time;          // Periodo de ciclo
      double        delay_time;          // Tempo proposto para start delay (offset)
      double        deadline_time;       // Tempo de limite maximo em fila
      unsigned int  payload_frame;       // Payload em Bytes
};


class Event
{
  public:
      double        time_intended;       // Tempo no qual o frame ira concorrer pelo barramento
      double        time_happened;       // Tempo real de acesso ao meio
      double        duration;            // Tempo de duração de escrita no barramento
      double        wcrt;                // Pior tempo de fila encontrado por esse frame
      double        deadline_occurred;   // Tempo do ultimo deadline ocorrido.
      bool          is_deadline;         // Variavel se diz se estou em tempo "deadline".
      Frame_CAN     frame;               // Dados do frame

      bool operator==(Event e) const{
          return (this->frame.id == e.frame.id);
      }
};


class Simulator_CAN
{
  public:
      std::vector<Event>   event_list;
      double               wcrt;
      double               time_mean_burst;
      double               frames_burst;
      unsigned int         deadlines;

      Simulator_CAN(Frame_CAN* frames, u_int16_t length);
      void run_simulation(double time_simulation);
  private:
     double                frames_mean;
     double                frames_mean_square;
     unsigned long int     num_queue;
     unsigned long int     num_frames;

     void add_event(Event new_event);
     void calc_time_burst(double time_current, bool length_queue);
     void realloc_event(Event new_event);
     void sort_list();
     Event get_priority_id();
};


Frame_CAN* get_CANDB(FILE* candb, u_int16_t& length);

#endif
