#include "simulador.hpp"
#include <unistd.h>
using namespace std;

// Quantidade de frames em fila na ultima vez.
static bool   last_queue = false;
// Time em que começou atual fila.
static double last_time  = 0;

Simulator_CAN::Simulator_CAN(Frame_CAN* frames, u_int16_t length)
{
    /* Limpa a lista. */
    this->event_list.clear();
    Event new_event;
    new_event.is_deadline                   = false;
    new_event.wcrt                          = 0;
    this->wcrt = this->frames_burst         = 0;
    this->time_mean_burst = this->deadlines = 0;
    /* Adiciona todos os Frames recebidos. */
    for (int i=0; i < length; i++)
    {
        /* Inicializa todos os campos. */
        new_event.time_intended = frames[i].delay_time;
        new_event.time_happened = frames[i].delay_time;
        new_event.frame         = frames[i];
        new_event.duration      = (((frames[i].payload_frame*8)+70)/DURATION_FRAME);
        /* Adiciona na lista. */
        this->add_event(new_event);
    }
    this->sort_list();
}

void Simulator_CAN::run_simulation(double time_simulation)
{
    Event  prioritary_frame;
    double time_current = 0;
    bool   cont_queue;
    this->num_queue     = 0;
    this->num_frames    = 0;
    this->frames_mean = this->frames_mean_square = 0;
    this->wcrt = this->frames_burst = this->time_mean_burst = this->deadlines = 0;

    last_queue = false;
    last_time  = 0;

    while (time_current < time_simulation)
    {
        /* Pega o ID mais prioritario de agora*/
        prioritary_frame = get_priority_id();

        /* Calcula o WCRT deste ID*/
        if (prioritary_frame.time_happened != prioritary_frame.time_intended)
           if (prioritary_frame.wcrt < (prioritary_frame.time_happened - prioritary_frame.time_intended))
              prioritary_frame.wcrt = (prioritary_frame.time_happened - prioritary_frame.time_intended);

        if (0)
        {
            system("clear");
            for(Event e: event_list)
                printf("ID: %4u \t CYCLE_TIME: %5f   \t CURRENT_TIME: %6f \t HAPPEN_TIME: %6f  \t WCRT: %2f\n",
                e.frame.id, e.frame.cycle_time, e.time_intended, e.time_happened, e.wcrt);
            printf("\n\nMAC ID: %4u \t TIME_WRITE: %f \t ATUAL_DELAY: %f \t MEU WCRT: %f \t DURATION_FRAME: %f\n", prioritary_frame.frame.id,
                  time_current, (double)(prioritary_frame.time_happened - prioritary_frame.time_intended), prioritary_frame.wcrt,
                  prioritary_frame.duration);
            getchar();
            // usleep(120000);
            printf("\n\n");
        }

        /* Avança do tempo*/
        time_current = prioritary_frame.time_happened+prioritary_frame.duration;

        /* Ajusta o tempo do novo evento*/
        prioritary_frame.time_happened += (prioritary_frame.frame.cycle_time + prioritary_frame.duration);
        prioritary_frame.time_intended  = prioritary_frame.time_happened;

        /* Adiciona o evento na lista de eventos*/
        this->realloc_event(prioritary_frame);

        /* Ajusta os frames concorrentes (LOST ARBITRATION) e verifica filas*/
        cont_queue = false;
        for (Event& e: this->event_list)
            if (e.time_happened < time_current)
            {
                e.time_happened = time_current;
                cont_queue      = true;
            }
            else break;

        /* Calcula maior time e frames no burst (fila)*/
        this->calc_time_burst(time_current, cont_queue);

        vector<Event> deadlines;
        deadlines.clear();
        /* Verifica deadlines */
        for (Event& e: this->event_list)
          if (e.is_deadline)
          {
              if ((e.time_happened - e.deadline_occurred) >= e.frame.deadline_time)
              {
                  e.time_happened = floor(time_current)+e.frame.cycle_time;
                  this->deadlines++;
                  e.deadline_occurred = e.time_happened;
                  deadlines.push_back(e);
              }
          }
          else
          {
              if ((e.time_happened - e.time_intended) >= e.frame.deadline_time)
              {
                  e.time_happened = floor(time_current)+e.frame.cycle_time;
                  this->deadlines++;
                  e.is_deadline = true;
                  e.deadline_occurred = e.time_happened;
                  deadlines.push_back(e);
              }
          }
        for (Event e: deadlines)
            this->realloc_event(e);
    }
    /* Calculo o pior WCRT (Acumulado de todos so wcrt)*/
    for (Event e: this->event_list)
      this->wcrt +=  e.wcrt;

    if (this->num_queue != 0)
    {
        this->time_mean_burst /= this->num_queue;
        this->frames_burst     = this->frames_mean/this->num_queue;
        double desvio          = ((this->frames_mean_square/this->num_queue)-(this->frames_burst*this->frames_burst));
        this->frames_burst    += k_chebychev*desvio;
        // printf("deadlines: %d\n", this->deadlines);
        // printf("k Desvio mais media: %f\n", this->frames_burst+(k_chebychev*desvio));
    }
}

void Simulator_CAN::add_event(Event new_event)
{
    // Adiciona se a fila for vazia
    if (!this->event_list.size()){
        this->event_list.push_back(new_event);
        return;
    }
    // Busca na fila a posição certa para inserção.
    for (vector<Event>::iterator it = this->event_list.begin(); it != this->event_list.end(); it++)
    {
        // Verifica se esta posição é maior.
        if (it->time_happened >= new_event.time_happened)
        {
            // Adiciona na posição antes do maior.
            this->event_list.insert(it, new_event);
            return;
        }
    }
    // Caso seja a maior de todas, adiciona no final.
    this->event_list.insert(this->event_list.end(), new_event);
}

void Simulator_CAN::calc_time_burst(double time_current, bool length_queue)
{
    // Quantidade de frames em fila na ultima vez.
    // static bool   last_queue = false;
    // Time em que começou atual fila.
    // static double last_time  = 0;

    /*
        Verifica existencia de colisão, e calcula statisticas do bust.
    */

    if (length_queue)
    {
        if (!last_queue)
        {
            last_time  = time_current;
            last_queue = true;
        }
        this->num_frames++;
    }
    else if (last_queue)
        {
            this->time_mean_burst    += (time_current-last_time);
            this->num_queue++;
            this->frames_mean        += this->num_frames;
            this->frames_mean_square += pow(this->num_frames,2);
            this->num_frames          = 0;

            last_time  = time_current;
            last_queue = false;
        }
}

Event Simulator_CAN::get_priority_id()
{
    unsigned int          prioritary_id = 0;

    for (size_t i = 1; i < this->event_list.size(); i++)
    {
        if (this->event_list[i].time_happened > this->event_list[prioritary_id].time_happened)
            break;

        if (this->event_list[i].frame.id < this->event_list[prioritary_id].frame.id)
            prioritary_id = i;
    }

    if (this->event_list[prioritary_id].is_deadline)
        this->event_list[prioritary_id].is_deadline = false;

    return this->event_list[prioritary_id];
}

void Simulator_CAN::realloc_event(Event new_event)
{
    int old_pos;
    int new_pos = -1;
    /*
        Localiza a posição antiga do evento
    */
    for (size_t i = 0; i < this->event_list.size(); i++)
        if (this->event_list[i].frame.id == new_event.frame.id)
        {
            old_pos = i;
            break;
        }

    /*
        Verifca se esta na ultima posição. se for apenas atualizar o evento.
    */
    if ((old_pos+1) == this->event_list.size())
    {
        this->event_list[old_pos] = new_event;
        return;
    }

    /*
        Verifica e pega o indice (já ajustado).
    */
    for (size_t i = (old_pos+1); i < this->event_list.size(); i++)
        if (this->event_list[i].time_happened >= new_event.time_happened)
        {
            new_pos = i-1;
            break;
        }

    // Verifica se não achou indice(sou o maior); Seta na ultima posição
    if (new_pos == -1)
        new_pos = this->event_list.size()-1;

    // Efetua SWAP.
    for (size_t i = old_pos; i < new_pos; i++)
        this->event_list[i] = this->event_list[i+1];

    this->event_list[new_pos] = new_event;
}

void Simulator_CAN::sort_list()
{
    unsigned int less_delay;
    for (size_t i = 0; i < this->event_list.size(); i++)
    {
        less_delay = i;
        for (size_t j = i+1; j < this->event_list.size(); j++)
        {
            if (this->event_list[less_delay].frame.delay_time > this->event_list[j].frame.delay_time)
                less_delay = j;

            if (less_delay != i)
            {
                Event aux = this->event_list[i];
                this->event_list[i] = this->event_list[less_delay];
                this->event_list[less_delay] = aux;
            }
        }
    }
}

Frame_CAN* get_CANDB(FILE* candb, u_int16_t& length)
{
   unsigned int  id;
   double        cycle_time, deadline_time, delay_time;
   unsigned int  payload_frame;

   Frame_CAN*    frames;
   length = 0;

   try
   {
      while (fscanf(candb,"%u\t%lf\t%lf\t%lf\t%u\n", &id, &cycle_time, &deadline_time, &delay_time, &payload_frame) != EOF)
      {
          if (frames)
          {
              frames = (Frame_CAN*) realloc(frames, sizeof(Frame_CAN)*(++length));
          }
          else
          {
              frames = (Frame_CAN*) malloc(sizeof(Frame_CAN)*(++length));
          }
          if (!frames)
          {
              std::cout << "\n[ERRO] Alocação de memoria para Frames na função get_CANDB()\n" << '\n';
              exit(ERRO_INPUT_CANDB);
          }
          frames[length-1].id            = id;
          frames[length-1].cycle_time    = cycle_time;
          frames[length-1].deadline_time = deadline_time;
          frames[length-1].delay_time    = delay_time;
          frames[length-1].payload_frame = payload_frame;
      }
   }
   catch(int e)
   {
       std::cout << "\n[ERRO] Leitura de frames do CANDB na função get_CANDB()\n" << '\n';
       exit(ERRO_INPUT_CANDB);
   }
   return frames;
}
