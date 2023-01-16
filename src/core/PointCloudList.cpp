/*
* Copyright (c) 2023 Carlos Tojal.
* All rights reserved.
*
* Doubly linked circular list of PointClouds implementation.
*/

#include "PointCloudList.h"

PointCloudList::PointCloudList() {

	// allocate the head
	this->head = (struct PointCloudListNode*) malloc(sizeof(struct PointCloudListNode));
	// with an empty list, head points to null
	head->next = nullptr;
	head->prev = nullptr;
	head->value = nullptr;
}

PointCloudList::~PointCloudList() {
	// when deleting the list, free every node to avoid memory leaks
	while(this->length > 0) {
		this->removeFromHead();
	}
}

size_t PointCloudList::getLength() {
	return length;
}

pcl::PointCloud<pcl::PointXYZ> *PointCloudList::removeFromHead() {
	pcl::PointCloud<pcl::PointXYZ> *removed = nullptr;

	// check if the list is not empty
	if(this->length == 0) {
		return nullptr;
	} else if(this->length == 1) { // only 1 element
		// this is a special case because when the list is empty the head points to nullptr

		removed = this->head->next->value;
		this->head->next = nullptr;

		this->length--;

		free(this->head->next);
		return removed;
	} else { // several elements
		
		struct PointCloudListNode *first = this->head->next;
		removed = first->value;
		struct PointCloudListNode *second = first->next;
		struct PointCloudListNode *tail = first->prev;

		this->head->next = second;
		second->prev = tail;
		
		this->length--;

		free(first);
		return removed;
	}


	return removed;
}

// remove a node by value
pcl::PointCloud<pcl::PointXYZ> *PointCloudList::remove(pcl::PointCloud<pcl::PointXYZ> *val) {
	if(this->length == 0)
			return nullptr;
	else if(this->length == 1)
			return this->removeFromHead();
	
	pcl::PointCloud<pcl::PointXYZ> *removed = nullptr;
	struct PointCloudListNode *toRemove = nullptr;

	struct PointCloudListNode *cur = this->head;
	do {
		// the next node is the one we are searching
		if(cur->next->value == val) {
			toRemove = cur->next;
			removed = toRemove->value;
			
			toRemove->next->prev = toRemove->prev;
			cur->next = toRemove->next;

			this->length--;

			free(toRemove);
			return removed;
		}
		cur = cur->next;
	} while(cur != this->head->next);

	return removed;
}

void PointCloudList::insertOnTail(pcl::PointCloud<pcl::PointXYZ> *cloud) {
	struct PointCloudListNode *new_node = nullptr;

	// allocate a new node
	if((new_node = (struct PointCloudListNode*) malloc(sizeof(struct PointCloudListNode))) == NULL) {
		std::cerr << "Error allocating the new list node: " << strerror(errno) << std::endl;
		return;
	}

	new_node->value = cloud;
	if(this->length == 0) {
		// the list is empty
		new_node->prev = new_node;
		new_node->next = new_node;
		this->head->next = new_node;
	} else {
			// the tail is the previous of the first
			struct PointCloudListNode *tail = this->head->next->prev;
			new_node->next = tail->next;
			tail->next = new_node;
			new_node->prev = tail;
	}

	this->length++;
}
