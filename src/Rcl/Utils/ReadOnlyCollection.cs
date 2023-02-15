using System.Collections;

namespace Rcl.Utils;

static class ReadOnlyCollection
{
    public static IReadOnlyCollection<T> Wrap<T>(ICollection<T> collection)
    {
        return new ReadOnlyCollectionWrapper<T>(collection);
    }

    public static IReadOnlyCollection<T> Wrap<T>(IReadOnlyCollection<T> collection)
    {
        return new ReadOnlyCollectionWrapper2<T>(collection);
    }
}

sealed class ReadOnlyCollectionWrapper<T> : IReadOnlyCollection<T>
{
    readonly ICollection<T> source;

    public ReadOnlyCollectionWrapper(ICollection<T> source) => this.source = source;

    public int Count => source.Count;

    public IEnumerator<T> GetEnumerator() => source.GetEnumerator();

    IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();
}

sealed class ReadOnlyCollectionWrapper2<T> : IReadOnlyCollection<T>
{
    readonly IReadOnlyCollection<T> source;

    public ReadOnlyCollectionWrapper2(IReadOnlyCollection<T> source) => this.source = source;

    public int Count => source.Count;

    public IEnumerator<T> GetEnumerator() => source.GetEnumerator();

    IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();
}