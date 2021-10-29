package dto

enum ResponseStatus {

    SUCCESS('success', 'Успех'),
    ERROR('error', 'Ошибка')

    private ResponseStatus(String id, String name) {
        this.id = id
        this.name = name
    }

    final String id
    final String name
}
