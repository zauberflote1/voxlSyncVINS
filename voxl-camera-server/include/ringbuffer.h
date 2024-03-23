/*******************************************************************************
 * Copyright 2022 ModalAI Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * 4. The Software is used solely in conjunction with devices provided by
 *    ModalAI Inc.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#ifndef RINGBUFFER_H
#define RINGBUFFER_H

template <typename T>
class DataNode {
    public:
        T data;
        DataNode<T> *next;
        DataNode<T> *prev;
};

template <typename T>
class RingBuffer {

    public:

        /**
         * @brief RingBuffer constructor
         *
         * @param size  number of Units in the buffer
         */
        RingBuffer(int size = 25) : size(size){
            current = (DataNode<T> *)calloc(sizeof(DataNode<T>), 1);
            current->next = current;
            current->prev = current; //prevent segfault on immediate deconstruction
        }

        // Free the ring
        ~RingBuffer() {
            current->prev->next = NULL;
            for(DataNode<T> *node = current; node != NULL;){
                DataNode<T> *tmp = node->next;
                free(node);
                node = tmp;
            }
        }

        /**
         * @brief generic insertertion function into the buf
         *
         * @param new_packet  pointer to BufUnit we want inserted
         *
         * @return < 0 on error, 0 on success
         */
        void insert_data(T new_packet){

            if(items_in_buf != size){

                DataNode<T> * newNode = (DataNode<T> *)calloc(sizeof(DataNode<T>), 1);
                newNode->next = current->next;
                current->next = newNode;

                newNode->prev = current;
                newNode->next->prev = newNode;

                items_in_buf++;
            }

            current->next->data = new_packet;
            current = current->next;

        }

        class Iterator
        {
          public:
            Iterator(DataNode<T> *ptr) {data = ptr;}

            T& operator*() const { return data->data; }
            T* operator->() { return &(data->data); }

            // Prefix increment
            Iterator& operator++() {
                data = data->prev;
                return *this;
            }

            // Postfix increment
            Iterator operator++(int) {
                Iterator tmp = *this;
                ++(*this);
                return tmp;
            }

            // Prefix decrement
            Iterator& operator--() {
                data = data->next;
                return *this;
            }

            // Postfix decrement
            Iterator operator--(int) {
                Iterator tmp = *this;
                --(*this);
                return tmp;
            }

            friend bool operator== (const Iterator& a, const Iterator& b) { return a.data == b.data; };
            friend bool operator!= (const Iterator& a, const Iterator& b) { return a.data != b.data; };

            private:

                DataNode<T> *data;

        };

        //Forward iterator is newest->oldest
        Iterator begin() {
            return Iterator(current);
        }
        Iterator end()   {
            return Iterator(current->next);
        }

        //Reverse iterator is oldest->newest
        Iterator rbegin() {
            return Iterator(current->next->next);
        }
        Iterator rend()   {
            return Iterator(current->next);
        }

    protected:

        /// how many units we have space for in the buf
        const int size;

        DataNode<T> *current;

        /// current index of the ring
        int index = 0;

        /// running count of number of items present in our buffer
        int items_in_buf = 0;

        /// mutex to protect our buffer
        std::mutex buf_mutex;

};

#endif //RINGBUFFER_H
